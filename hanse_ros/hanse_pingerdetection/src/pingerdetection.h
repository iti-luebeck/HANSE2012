#include <ros/ros.h>
#include <ros/network.h>
#include <QString>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <cmath>

#include <QtCore>
#include <QtMultimediaKit/QAudio>
#include <QtMultimediaKit/QAudioFormat>
#include <QtMultimediaKit/QAudioInput>
#include <QtMultimediaKit/QAudioDeviceInfo>
#include <QIODevice>

#define STATE_WAIT_FOR_FIRST_SIGNAL "Wait for signal"
#define STATE_COUNT_DELAY "Count delay between both micros"
#define STATE_WAIT_FOR_SIGNAL_FINISHED "Wait until no signal is detected"
#define STATE_SAVE_DATA "Save data"

#define STATE_WAIT_FOR_LEFT_MICRO_SIGNAL "Wait left micro signal"
#define STATE_WAIT_FOR_RIGHT_MICRO_SIGNAL "Wait right micro signal"

#define STATE_BOTH_MICRO_SIGNAL_DETECTED "Both micros got a signal at the same time"
#define STATE_LEFT_MICRO_SIGNAL_DETECTED "Left micro got a signal first"
#define STATE_RIGHT_MICRO_SIGNAL_DETECTED "Right micro got a signal first"

using namespace QtMultimediaKit;

namespace hanse_pingerdetection{

class PingerDetection : public QThread {

public:
    PingerDetection();
    virtual ~PingerDetection();
    void run();

public slots:
    void receiveAudio();

private:
    // Methods

    void setEnabled(bool b);

    void setRecordSource(QString n);

    void pingerDetectionCallback(const std_msgs::StringConstPtr &msg);

    void initVariables();
    void configAudio();

    void audioProcessing();

    double leftNonCoherentFSKDemodulator(double x, bool skipPop);
    double rightNonCoherentFSKDemodulator(double y, bool skipPop);

    void periodicSinCos();
    void inkrementSinCosCounter();

    void signalDelayAnalysis(double goertzelLeft, double goertzelRight);
    void calculateAngle(int samplediff);

    // ROS

    ros::NodeHandle nh;
    ros::Subscriber input_subscriber;
    ros::Publisher angle_publisher;
    ros::Publisher msg_publisher;
    ros::Publisher left_publisher;
    ros::Publisher right_publisher;

    // Variables

    bool enabled;

    int noiseLeft;
    int noiseRight;
    double angle;


    QAudioInput *audioInput;
    QIODevice *ioDevice;

    QFile *outputFile;
    QTextStream* outputStream;

    QFile *outputFileFSK;
    QTextStream* outputStreamFSK;


    QFile *inputFile;
    QTextStream* inputStream;

    QList<double> saveListLeftMicroInt;
    QList<double> saveListRightMicroInt;
    QList<int> saveListTime;

    QList<double> saveListLeftFSK;
    QList<double> saveListRightFSK;

    QList<double> saveListAngle;

    QList<int> inputTime;
    QList<double> inputLeftMicroInt;
    QList<double> inputRightMicroInt;

    QString recordSource;

    QByteArray audioBuffer;
    QAudioFormat audioFormat;

    QList<int> decodeData(const char *data, qint64 len);

    quint16 audioMaxAmplitude;

    int sampleCounter;

    QList<double> leftMicro;
    QList<double> rightMicro;

    QList<double> leftMicroCos;
    QList<double> leftMicroSin;

    QList<double> rightMicroCos;
    QList<double> rightMicroSin;



    double integralSinLeft;
    double integralCosLeft;

    double integralSinRight;
    double integralCosRight;

    QList<double> cosLUT;
    QList<double> sinLUT;
    int cosCounter;
    int sinCounter;



    int delayAverageElements;

    QString delayState;
    QString delaySubstate;
    int leftMicroSampleDelay;

    int leftMicroSignalDetected;
    int rightMicroSignalDetected;
    int signalDetected;

    int rightMicroSampleDelay;
    int bothMicroSignalDetectedCount;

    double leftMicroMaxMagnitude;
    double rightMicroMaxMagnitude;

    double leftAverageThreshold;
    double rightAverageThreshold;

    double leftFSK;
    double rightFSK;


    // Config
    double omega;
    int sampleRate;
    int window;
    double scale;
    double threshold;
};
}
