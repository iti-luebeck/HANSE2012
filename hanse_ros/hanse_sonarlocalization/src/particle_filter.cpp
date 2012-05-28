#include <algorithm>
#include "util.h"
#include "particle_filter.h"
#include <trng/mt19937.hpp>
#include <trng/normal_dist.hpp>

ParticleFilter::ParticleFilter(hanse_sonarlocalization::ParticleFilterConfig config, Params params) :
    config(config),
    worldMap(params.map_image, params.map_pixel_size, params.map_threshold)
{
    imuInitialized = false;
    accelerationMean = Eigen::Vector2f(0, 0);
}

void ParticleFilter::reconfigure(hanse_sonarlocalization::ParticleFilterConfig config)
{
    this->config = config;
    if (particles.size() != (unsigned)config.particle_count) {
	resetPosition();
    }
}

Eigen::ArrayXXf ParticleFilter::randNormal(int m, int n, float sigma, float mu) {
    static trng::mt19937 R;
    trng::normal_dist<float> normal(mu, sigma);

    Eigen::ArrayXXf X(m,n);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
	    X(i, j) = normal(R);
	}
    }
    return X;
}

void ParticleFilter::resetPosition()
{
    Eigen::Array2f mapSize = worldMap.mapSize();
    Eigen::ArrayXXf randomPositions = (Eigen::ArrayXXf::Random(2, config.particle_count) + 1) / 2;
    Eigen::ArrayXXf randomRotations = Eigen::ArrayXXf::Random(1, config.particle_count) * M_PI;

    particles.clear();

    for (int i = 0; i < config.particle_count; i++) {
	Particle particle;
	particle.weight = 1.0;
	particle.velocity = Eigen::Vector2f(0, 0);
	particle.position = Eigen::Translation<float, 2>(randomPositions.col(i) * mapSize) * Eigen::Rotation2D<float>(randomRotations(0, i));
	particles.push_back(particle);
    }
}

void ParticleFilter::setPosition(Eigen::Affine2f position)
{
    /*
    Particle p;
    p.weight = 1.0;
    p.position = position;
    p.velocity = Eigen::Vector2f(0, 0);
    particles.clear();
    particles.resize(config.particle_count, p);
    */

    Eigen::ArrayXXf randomRotations = Eigen::ArrayXXf::Random(1, config.particle_count) * M_PI;

    particles.clear();

    for (int i = 0; i < config.particle_count; i++) {
	Particle particle;
	particle.weight = 1.0;
	particle.velocity = Eigen::Vector2f(0, 0);
	particle.position = Eigen::Translation<float, 2>(position.translation()) * Eigen::Rotation2D<float>(randomRotations(0, i));
	particles.push_back(particle);
    }

}

void ParticleFilter::move(float seconds)
{
    for (auto &particle : particles) {
	particle.position.pretranslate(particle.velocity * seconds);
    }
}

void ParticleFilter::perturb()
{
    Eigen::MatrixXf distance = randNormal(2, config.particle_count, config.perturb_position, 0);
    Eigen::MatrixXf acceleration = randNormal(2, config.particle_count, config.perturb_velocity, 0);
    Eigen::MatrixXf rotation = randNormal(1, config.particle_count, config.perturb_rotation, 0);
    for (int i = 0; i < config.particle_count; i++) {
	particles[i].velocity = particles[i].velocity + Eigen::Vector2f(acceleration.col(i));
	particles[i].position.pretranslate(Eigen::Vector2f(distance.col(i)));
	particles[i].position.rotate(rotation(0, i));
    }
}

void ParticleFilter::weightParticles(const hanse_msgs::WallDetection &msg)
{
    bestParticle.weight = -1;
    for (auto &particle : particles) {
	weightParticle(particle, msg);
    }
    //ROS_INFO("best particle weight %f", bestParticle.weight);
}

void ParticleFilter::resample()
{
    int particle_count = particles.size();

    float sum = 0;
    for (auto &particle : particles) {
	sum += particle.weight;
    }
    if (sum == 0 || isnan(sum)) {
	ROS_INFO("zero or NaN weights: %f", sum);
	return;
    }
    float eff = 0;
    for (auto &particle : particles) {
	eff += (particle.weight * particle.weight) / (sum * sum);
    }

    if (2 * eff >= particle_count) {
	ROS_INFO("efficient enough");
	return;
    }

    std::vector<float> commulativeWeights;
    commulativeWeights.reserve(particle_count);

    float last = 0;
    for (auto &particle : particles) {
	last = particle.weight / sum + last;
	commulativeWeights.push_back(last);
    }

    Eigen::ArrayXXf rand = (Eigen::ArrayXXf::Random(1, config.particle_count) + 1) / 2;

    ParticleVector newParticles;
    newParticles.reserve(config.particle_count);

    for (int i = 0; i < config.particle_count; i++) {
	auto iterator = std::lower_bound(commulativeWeights.begin(), commulativeWeights.end(), rand(0, i));
	int position = iterator - commulativeWeights.begin();
	if (position >= particle_count)
	    position = particle_count - 1;
	newParticles.push_back(particles[position]);
    }

    particles = std::move(newParticles);
}

Eigen::Affine2f ParticleFilter::estimatedPosition()
{
    Eigen::Vector2f meanPosition(0, 0);
    Eigen::Vector2f meanDirection(0, 0);
    for (auto &particle : particles) {
	meanPosition += particle.position.translation();
	meanDirection += particle.position.rotation() * Eigen::Vector2f(1, 0);
    }
    meanPosition /= particles.size();
    double meanAngle = atan2f(meanDirection.y(), meanDirection.x());

    return Eigen::Translation2f(meanPosition) * Eigen::Rotation2D<float>(meanAngle);
}

void ParticleFilter::weightParticle(Particle &particle, const hanse_msgs::WallDetection &msg)
{
    float sigma = config.weight_sigma;
    float weight = 1;
    float auvDistance = worldMap.wallDistance(particle.position.translation());
    float weight_scaling = config.weight_scaling;

    if (auvDistance < 0) {
	weight = 1e-10;
    } else if (msg.wallDetected && !msg.distances.empty()) {
	Eigen::Rotation2D<float> rotation(msg.headPosition);

	Eigen::Translation<float, 2> translation(1, 0);
	Eigen::Affine2f wallPosition = particle.position.rotation() * rotation * translation;

	// expected wall distance
	float expectedDistance =
	    worldMap.directedWallDistance(particle.position.translation(),
					  wallPosition.translation(),
					  2 * msg.range);

	float distance = HUGE_VAL;
	for (float range : msg.distances)
	    distance = std::min(distance, fabsf(expectedDistance - range));

	distance -= config.bend_distance;
	if (distance < 0)
	    distance /= config.bend_factor;
	distance += config.bend_distance / config.bend_factor;

	weight *= weight_scaling + (1 - weight_scaling) * expf( - 0.5 * powf(distance / sigma, 2));
    }
    particle.weight = weight;
    if (particle.weight > bestParticle.weight) {
	bestParticle = particle;
    }
}

void ParticleFilter::addImuMessage(sensor_msgs::Imu const &imu)
{
    Eigen::Quaternionf orientationFlipped(imu.orientation.w,
					  imu.orientation.x,
					  imu.orientation.y,
					  imu.orientation.z);

    // This undos the rotation of the xsens (180 deg around the y
    // axis) this is why: we first (right side) put the xsens into hanse the
    // wrong way around and _then_ move hanse around (left side) so
    // the inverse of the xsens orientation has to be at the right
    // side too for them to cancle
    Eigen::Quaternionf orientation = orientationFlipped * Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(0, 1, 0));

    // We assume that all particles have this orientation (i. e. the
    // last orientation we used to update them, projected onto the
    // xy-plane)
    float lastTheta = localization::thetaFromQuaternion(lastImuOrientation);
    Eigen::Quaternionf particleBase;
    particleBase = Eigen::AngleAxis<float>(lastTheta, Eigen::Vector3f(0, 0, 1));

    Eigen::Vector3f accelerationAUV(imu.linear_acceleration.x,
				    imu.linear_acceleration.y,
				    imu.linear_acceleration.z);


    // the acceleration is in AUV space, so we transform it into world
    // space first and then into particle space
    Eigen::Vector3f accelerationParticle = particleBase.inverse() * orientationFlipped * accelerationAUV;

    // When we arrived in particle space we can discard the z
    // component (this will only be rotated around the z axis, and the
    // z component itself won't be used again)
    Eigen::Vector2f accelerationParticle2D = Eigen::Vector2f(accelerationParticle.x(),
							     accelerationParticle.y());


    if (config.imu_motion) {
        ROS_INFO("particle acceleration: %8.5f %8.5f", accelerationParticle2D.x(), accelerationParticle2D.y());

        ROS_INFO("particle acceleration mean: %8.5f %8.5f", accelerationMean.x(), accelerationMean.y());
    }

    // To do anything usefull with the acceleration we need to know the time step
    float dt = (imu.header.stamp - lastImuMsgTime).toSec();

    // TODO: better results might be obtained below by using the midpoint
    // method or similar if applicable (check this)

    // We integrate the velocity once and the acceleration twice to obtain the offset
    offsetParticle += dt * velocityParticle + 0.5 * dt * dt * accelerationParticle2D;
    // We integrate the acceleration to obtain the velocity
    velocityParticle += dt * accelerationParticle2D;

    imuOrientation = orientation;

    lastImuMsgTime = imu.header.stamp;
}

void ParticleFilter::imuUpdate()
{
    if (imuInitialized) {
	// Doing the projection onto the 2d plane first should be more
	// accurate when pitch and roll are nonzero
	float lastTheta = localization::thetaFromQuaternion(lastImuOrientation);
	float theta = localization::thetaFromQuaternion(imuOrientation);

	float deltaTheta = theta - lastTheta;

        if (config.imu_motion) {
            ROS_INFO("particle velocity: %8.5f %8.5f", velocityParticle.x(), velocityParticle.y());
            ROS_INFO("particle offset: %8.5f %8.5f", offsetParticle.x(), offsetParticle.y());
        }

	if (config.imu_motion) {
	    for (auto &particle : particles) {
		// velocity delta is relative to our current orientation
		particle.velocity += particle.position.rotation() * velocityParticle;
		// we have our current position and orientation, add the
		// offset to that and _then_ turn the particle
		particle.position = particle.position *
		    Eigen::Translation2f(offsetParticle) * Eigen::Rotation2D<float>(deltaTheta);
	    }
	} else {
            ROS_INFO("delta theta: %f", deltaTheta);
	    for (auto &particle : particles) {
		particle.position = particle.position * Eigen::Rotation2D<float>(deltaTheta);
	    }
	}
    }

    lastImuOrientation = imuOrientation;
    velocityParticle = Eigen::Vector2f(0, 0);
    offsetParticle = Eigen::Vector2f(0, 0);
    imuInitialized = true;
}

void ParticleFilter::thrusterUpdate(double time, int leftSpeed, int rightSpeed)
{
    if (!config.thruster_motion)
        return;
    float left = leftSpeed / 127.f;
    float right = rightSpeed / 127.f;

    float mid = (left + right) / 2;

    float speed = mid * config.thruster_speed;

    float mu = expf(-time * config.thruster_rate);

    float muC = 1 - mu;

    float speedS = speed * muC;

    for (auto &particle : particles) {
        particle.velocity = (mu * particle.velocity + particle.position.rotation() * Eigen::Vector2f(speedS, 0.f));
    }
}
