#include "pingerdetection.h"
#include <QApplication>

namespace hanse_pingerdetection{

void PingerDetection::run()
{
    QEventLoop eventLoop;
    eventLoop.processEvents();

    outputFile = new QFile("/tmp/pingerdetection"+QString::number(lognr)+".record");
    outputFile->remove();

    if (outputFile->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream = new QTextStream(outputFile);
        ROS_INFO("Created file %s", outputFile->fileName().toStdString().c_str());
    } else {
        ROS_INFO("Could not open file to write %s", outputFile->fileName().toStdString().c_str());
    }

    ioDevice = audioInput->start();

    ROS_INFO("Skip first samples...");
    // Komische Ausschlaege am Anfang einer Messung uebergehen
    while(sampleCounter < 2000000 && ros::ok() && enabled){
        qint64 bytesReady = audioInput->bytesReady();
        qint64 l = ioDevice->read(audioBuffer.data(), bytesReady);
        Q_UNUSED(l)
        sampleCounter++;
    }
    ROS_INFO("Skipped first samples");
    sampleCounter = 0;

    QTime timestamp;
    timestamp.restart();


    leftMicroSin.reserve(window);
    leftMicroCos.reserve(window);
    rightMicroSin.reserve(window);
    rightMicroCos.reserve(window);

    for(int i = 0; i < window; i++){
        leftMicroSin.append(0);
        leftMicroCos.append(0);
        rightMicroSin.append(0);
        rightMicroCos.append(0);
    }

    while (ros::ok() && enabled) {

        if(!audioInput){
            ROS_ERROR("No audio input");
            return;
        }

        // Bereitstehende Daten ermitteln und anschließend ggf. lesen
        QByteArray newData;
        qint64 l = 0;

        while((int)l == 0) {
            usleep(1000 / 400);
            ros::spinOnce();
            newData = ioDevice->readAll();
            l = newData.length();
        }

        //ROS_INFO("l %i", (int)l);
        // Neue Daten von der Soundkarte decodieren
        QList<int> decodeList;
        decodeList.append(this->decodeData(newData.constData(), l));
        //ROS_INFO("decodeList.length %i", decodeList.length());

        if(!decodeList.isEmpty()){

            double zerolineLeft = 0.0;
            double zerolineRight = 0.0;

            for(int i = 0; i < decodeList.length()-2; i+=2){

                // Decodierte Daten von Rauschen befreien...
                zerolineLeft = ((double)decodeList.at(i)-(double)noiseLeft)*scale*fetteSkalierung;
                zerolineRight = ((double)decodeList.at(i+1)-(double)noiseRight)*scale;

                if(plotRaw){
                    audioPlotRaw.addSample(zerolineLeft, zerolineRight);
                }

                if(saveData){
                    *outputStream << (int)(100000 * zerolineLeft) << ", " << (int)(100000 * zerolineRight) << "," << (int)(100000 *leftFSK)  << ", " << (int)(100000 *rightFSK) << ", " << (int)(100000 *angle) << "\r\n";
                }

                sampleCounter++;

                // Ermittelte Daten verarbeiten (auf Reihenfolge der Counter achten!)
                this->audioProcessing(zerolineLeft, zerolineRight);

                listCounter++;
                if(listCounter == window){
                    listCounter = 0;
                }
            }

        }
    }

    ROS_INFO("Samples %i", sampleCounter);

    outputStream->flush();


    ROS_INFO("Pingerdetection finished stop");
    audioInput->stop();
}


void PingerDetection::initVariables(){
    ROS_INFO("Init pinger detection variables");

    sampleCounter = 0;

    integralSinLeft = integralCosLeft = integralSinRight = integralCosRight = 0.0;

    listCounter = 0;

    delayAverageElements = 0;
    leftAverageThreshold = rightAverageThreshold = 0.0;

    bothMicroSignalDetectedCount = 0;
    leftMicroSampleDelay = rightMicroSampleDelay = 0;
    leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

    delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
    cosCounter = sinCounter = 0;
    leftFSK = rightFSK = 0.0;

    leftMicroSignalDetected = rightMicroSignalDetected = 0;

    noiseLeft = noiseRight = 0;

    angle = 0.0;

    leftMicroSin.clear();
    leftMicroCos.clear();
    rightMicroSin.clear();
    rightMicroCos.clear();

    leftMicroAverageMagnitude = rightMicroAverageMagnitude = 0.0;
    leftMicroAvgCounter = rightMicroAvgCounter = 0;
}

void PingerDetection::configAudio(){
    ROS_INFO("Config audio device");

    QAudioDeviceInfo info;
    info = QAudioDeviceInfo::defaultInputDevice();

    QList<QAudioDeviceInfo> list = info.availableDevices(QAudio::AudioInput);
    for(int i = 0; i < list.length(); i++){
        QString deviceTemp = list.at(i).deviceName();
        ROS_INFO("Devices... %s", deviceTemp.toStdString().c_str());
    }

    QList<int> supSR = info.supportedSampleRates();
    ROS_INFO("Supported sample rates:");

    for(int i = 0; i < supSR.length(); i++){
        ROS_INFO("%i", supSR.at(i));
    }

    QStringList supportedCodecs = info.supportedCodecs();
    ROS_INFO("Supported codecs: ");
    for(int i = 0; i < supportedCodecs.length(); i++){
        ROS_INFO("%s", supportedCodecs.at(i).toStdString().c_str());
    }

    audioBuffer.resize(sampleRate);
    ROS_INFO("Audio buffer length %i", audioBuffer.length());
    // set up the format you want, eg.
    audioFormat.setFrequency(sampleRate);
    // Mono / Stereo
    audioFormat.setChannels(2);

    // 16 Bit
    audioFormat.setSampleSize(16);
    audioFormat.setCodec("audio/pcm");
    // Big / Little endian
    audioFormat.setByteOrder(QAudioFormat::LittleEndian);
    audioFormat.setSampleType(QAudioFormat::SignedInt);

    ROS_INFO("Device sample rate %i", audioFormat.sampleRate());
    ROS_INFO("Device sample size %i", audioFormat.sampleSize());
    ROS_INFO("Device codec %s", audioFormat.codec().toStdString().c_str());


    if(!info.isFormatSupported(audioFormat)) {
        ROS_INFO("Format not supported - try to use nearest");
        audioFormat = info.nearestFormat(audioFormat);

        ROS_INFO("Device sample rate %i", audioFormat.sampleRate());
        ROS_INFO("Device sample size %i", audioFormat.sampleSize());
        ROS_INFO("Device codec %s", audioFormat.codec().toStdString().c_str());

        QStringList supportedCodecs = info.supportedCodecs();
        ROS_INFO("Supported codecs: ");
        for(int i = 0; i < supportedCodecs.length(); i++){
            ROS_INFO("%s", supportedCodecs.at(i).toStdString().c_str());
        }

    }

    ROS_INFO("Used device: %s", info.deviceName().toStdString().c_str());

    audioInput = new QAudioInput(audioFormat, this);

}

void PingerDetection::setRecordSource(QString n){
    recordSource = n;
    inputFile = new QFile(recordSource+".record");

    if (inputFile->open(QFile::ReadOnly | QIODevice::ReadOnly)) {
        inputStream = new QTextStream(inputFile);
    } else {
        ROS_INFO("Could not open file to read %s",inputFile->fileName().toStdString().c_str());
    }
}

QList<int> PingerDetection::decodeData(const char *data, qint64 len){
    // Decodierung des PCM Signals

    QList<int> decodeDataList;

    Q_ASSERT(audioFormat.sampleSize() % 8 == 0);
    const int channelBytes = audioFormat.sampleSize() / 8;
    const int sampleBytes = audioFormat.channels() * channelBytes;
    Q_ASSERT(len % sampleBytes == 0);
    const int numSamples = len / sampleBytes;

    const unsigned char *ptr = reinterpret_cast<const unsigned char *>(data);

    for (int i = 0; i < numSamples; i++) {
        for(int j = 0; j < audioFormat.channels(); j++) {
            quint16 value = 0;

            if (audioFormat.sampleSize() == 8 && audioFormat.sampleType() == QAudioFormat::UnSignedInt) {
                value = *reinterpret_cast<const quint8*>(ptr);
            } else if (audioFormat.sampleSize() == 8 && audioFormat.sampleType() == QAudioFormat::SignedInt) {
                value = qAbs(*reinterpret_cast<const qint8*>(ptr));
            } else if (audioFormat.sampleSize() == 16 && audioFormat.sampleType() == QAudioFormat::UnSignedInt) {
                if (audioFormat.byteOrder() == QAudioFormat::LittleEndian)
                    value = qFromLittleEndian<quint16>(ptr);
                else
                    value = qFromBigEndian<quint16>(ptr);
            } else if (audioFormat.sampleSize() == 16 && audioFormat.sampleType() == QAudioFormat::SignedInt) {
                if (audioFormat.byteOrder() == QAudioFormat::LittleEndian)
                    value = qAbs(qFromLittleEndian<qint16>(ptr));
                else
                    value = qAbs(qFromBigEndian<qint16>(ptr));
            }

            decodeDataList.append(value);

            ptr += channelBytes;
        }


    }
    return decodeDataList;
}

void PingerDetection::audioProcessing(double leftRaw, double rightRaw){
    ROS_DEBUG("Audio processing");

    if(sampleCounter >= window){
        // Gewnschte Fensterlnge wurde erreicht

        // Daten verarbeiten
        leftFSK =  leftNonCoherentFSKDemodulator(leftRaw, false);
        rightFSK =  rightNonCoherentFSKDemodulator(rightRaw, false);

        if(plotGoertzel){
            audioPlotGoertzel.addSample(leftFSK, rightFSK);
        }

        // Sin/Cos Zaehler erhhen
        inkrementSinCosCounter();

        // FSK Werte verwursten
        signalDelayAnalysis(leftFSK, rightFSK);

        angle_publisher.publish(winkel);

    } else {
        // Integral schrittweise bilden
        leftNonCoherentFSKDemodulator(leftRaw, true);
        rightNonCoherentFSKDemodulator(rightRaw, true);
        inkrementSinCosCounter();

    }
}

void PingerDetection::signalDelayAnalysis(double left, double right){

    // Ueberpruefen, ob ein Mikro ein Signal groesser als den Schwellwert aufweist
    // 1 = true
    // 0 = false
    if(left > threshold){
        leftMicroSignalDetected = 1;

    } else {
        leftMicroSignalDetected = 0;
    }

    if(right > threshold){
        rightMicroSignalDetected = 1;
    } else {
        rightMicroSignalDetected = 0;
    }


    if (timeoutCounter != 0) {
        timeoutCounter--;
        if (timeoutCounter == 0) {
            if (delayState == STATE_WAIT_FOR_LEFT_MICRO_SIGNAL ||
                delayState == STATE_WAIT_FOR_RIGHT_MICRO_SIGNAL) {
                delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;
            }
        }
    }



    if(delayState == STATE_WAIT_FOR_FIRST_SIGNAL){

        signalDetected = leftMicroSignalDetected + rightMicroSignalDetected;

        if(signalDetected >= 1) {
            timeoutCounter = sampleRate / 10;
        }

        if(signalDetected == 2){
            // Beide Mikros haben ein Signal goersser als den Schwellwert detektiert
            // Keine messbare Zeitverzoegerung

            bothMicroSignalDetectedCount++;

            leftMicroMagnitudeCalculation(left);
            rightMicroMagnitudeCalculation(right);

            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else if(signalDetected == 1){

            if(leftMicroSignalDetected == 1){
                // Linkes Mikro hat ein Signal grer als den Schwellwert
                delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

                leftMicroMagnitudeCalculation(left);

                // Rechtes Mikro hat ein Delay
                rightMicroSampleDelay++;

            } else if(rightMicroSignalDetected == 1){
                // Rechtes Mikro hat ein Signal grer als den Schwellwert
                delayState = STATE_WAIT_FOR_LEFT_MICRO_SIGNAL;

                rightMicroMagnitudeCalculation(right);

                // Linkes Mikro hat ein Delay
                leftMicroSampleDelay++;

            }
        }
    } else if(delayState == STATE_WAIT_FOR_LEFT_MICRO_SIGNAL){

        if(left > threshold){
            leftMicroSignalDetected = 1;

            leftMicroMagnitudeCalculation(left);

            bothMicroSignalDetectedCount++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            leftMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            leftMicroSampleDelay++;
        }

        rightMicroMagnitudeCalculation(right);


    } else if(delayState == STATE_WAIT_FOR_RIGHT_MICRO_SIGNAL){

        if(right > threshold){
            rightMicroSignalDetected = 1;

            rightMicroMagnitudeCalculation(right);

            bothMicroSignalDetectedCount++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            rightMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            rightMicroSampleDelay++;
        }

        leftMicroMagnitudeCalculation(left);

    } else if(delayState == STATE_WAIT_FOR_SIGNAL_FINISHED){

        // Bisschen Toleranz bei der Abklingzeit um leichte Schwingungen nicht ins Gewicht fallen zu lassen.
        if(left < threshold*0.8){
            leftMicroSignalDetected = 0;
        } else {
            leftMicroSignalDetected = 1;
        }

        if(right < threshold*0.8){
            rightMicroSignalDetected = 0;
        } else {
            rightMicroSignalDetected = 1;
        }

        signalDetected = leftMicroSignalDetected + rightMicroSignalDetected;

        leftMicroMagnitudeCalculation(left);

        rightMicroMagnitudeCalculation(right);

        if(signalDetected == 2){

            bothMicroSignalDetectedCount++;

        } else if(signalDetected == 0){

            //////////////////////////////////////////////////////////////////////////////////////////////////
            // leftMircoSampleDelay > rightMicroSampleDelay => Der Pinger befindet sich rechts
            //      => calculateAngle erhält einen positiven Wert
            // leftMircoSampleDelay < rightMicroSampleDelay => Der Pinger befindet sich links
            //      => calculateAngle erhält einen negativen Wert
            //////////////////////////////////////////////////////////////////////////////////////////////////
            if (leftMicroAvgCounter == 0) {
                leftMicroAverageMagnitude = 0;
            } else {
                leftMicroAverageMagnitude /= leftMicroAvgCounter;
            }

            if (rightMicroAvgCounter == 0) {
                rightMicroAverageMagnitude = 0;
            } else {
                rightMicroAverageMagnitude /= rightMicroAvgCounter;
            }

            double delay;

            if(leftMicroSampleDelay > rightMicroSampleDelay){
                if(plotAnalysis){
                    ROS_INFO("Verzoegerung %i -> Rechtsdrehung", leftMicroSampleDelay);
                }
                calculateAngle(leftMicroSampleDelay);
                delay = -leftMicroSampleDelay / (double)sampleRate;

            } else if (leftMicroSampleDelay < rightMicroSampleDelay){
                if(plotAnalysis){
                    ROS_INFO("Verzoegerung %i -> Linksdrehung", rightMicroSampleDelay*(-1));
                }
                calculateAngle(rightMicroSampleDelay*-1);
                delay = rightMicroSampleDelay / (double)sampleRate;

            } else if (leftMicroSampleDelay == rightMicroSampleDelay){
                if(plotAnalysis){
                    ROS_INFO("Keine Verzoegerung -> Geradeaus");
                }
                calculateAngle(0);
                delay = 0;
            }

            timeoutCounter = 0;

            hanse_msgs::PingerDetection msg;
            msg.header.stamp = ros::Time::now();
            msg.leftAmplitude = leftMicroAverageMagnitude;
            msg.rightAmplitude = rightMicroAverageMagnitude;
            msg.timeDifference = delay;
            msg.angle = angle;
            
            angle_publisher.publish(msg);

            delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
            bothMicroSignalDetectedCount = 0;
            leftMicroSampleDelay = rightMicroSampleDelay = 0;
            leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

            leftMicroAverageMagnitude = rightMicroAverageMagnitude = 0.0;
            leftMicroAvgCounter = rightMicroAvgCounter = 0;
        }
    }
}

void PingerDetection::leftMicroMagnitudeCalculation(double left){
    if(left > leftMicroMaxMagnitude){
        leftMicroMaxMagnitude = left;
    }
    leftMicroAverageMagnitude += left;
    leftMicroAvgCounter++;
}

void PingerDetection::rightMicroMagnitudeCalculation(double right){
    if(right > rightMicroMaxMagnitude){
        rightMicroMaxMagnitude = right;
    }
    rightMicroAverageMagnitude += right;
    rightMicroAvgCounter++;
}

void PingerDetection::calculateAngle(int samplediff){

    ROS_DEBUG("Calcualte angle");
    if(samplediff == 0){
        winkel.data = 0.0;

    } else {

        double baseline = 0.4; // Distanz zwischen den Hydrophonen in m

        double delta_t = 48000/samplediff; // Berechnung des Zeitunterschieds in Sekunden

        double distdiff = 1500 / delta_t; // Berechnung der zurückgelegten Wegstrecke aus dem Zeitunterschied.

        angle = distdiff / baseline;
        winkel.data = angle;
        if(plotAnalysis){
            ROS_INFO("Winkel %f", angle);
        }
    }
}

// ---------------------------------- nichtkohrenter FSK-Demodulator

void PingerDetection::periodicSinCos(){

    sinLUT.clear();
    cosLUT.clear();
    sinCounter = 0;
    cosCounter = 0;

    // Periodische LUT erstellen fr 48k Hz Abtastfrequenz.

    // sin(0) = 0 und sin(24) = 0 (fast)
    for(int i = 0; i < 24; i++){
        double sinTemp = sin(2.0*M_PI*omega*(double)i/(double)sampleRate);
        sinLUT.append(sinTemp);
    }

    // cos(0) = 1 und cos(48) = 1
    for(int i = 0; i < 48; i++){
        double cosTemp = cos(2.0*M_PI*omega*(double)i/(double)sampleRate);
        cosLUT.append(cosTemp);
    }

    //    for(int x = 0; x < 70;  x){
    //        //qDebug() << "x= "<< x;
    //        double sinLeft = sin(2.0*M_PI*(double)omega*(double)sinCounter/(double)sampleRate);
    //        double cosLeft = cos(2.0*M_PI*(double)omega*(double)cosCounter/(double)sampleRate);
    //        //qDebug() << "sinLeft " << sinLeft << "   cosLeft " << cosLeft;
    //         inkrementSinCosCounter();
    //    }

}

void PingerDetection::inkrementSinCosCounter(){
    cosCounter++;
    sinCounter++;
    // Zhler ggf. zurck setzen, falls die Nachschlagtabellen vollstndig abgearbeitet wurden
    if(cosCounter == cosLUT.length()){
        cosCounter = 0;
    }
    if(sinCounter == sinLUT.length()){
        sinCounter = 0;
    }
}

double PingerDetection::leftNonCoherentFSKDemodulator(double x, bool skipPop){
    // Linkes Mikrofon: Integral abndern und Magnitude berechnen

    double sinLeftPush = sinLUT.at(sinCounter)*x;
    double cosLeftPush = cosLUT.at(cosCounter)*x;
    leftMicroSin.replace(listCounter, sinLeftPush);
    leftMicroCos.replace(listCounter, cosLeftPush);

    integralSinLeft = sinLeftPush;
    integralCosLeft = cosLeftPush;

    if(!skipPop){
        double sinLeftPop = leftMicroSin.at(window-1-listCounter);
        double cosLeftPop = leftMicroCos.at(window-1-listCounter);

        integralSinLeft -= sinLeftPop;
        integralCosLeft -= cosLeftPop;

        // Magnitude
        return pow(integralSinLeft, 2.0)+pow(integralCosLeft, 2.0);
    } else {

        return 0.0;
    }
}


double PingerDetection::rightNonCoherentFSKDemodulator(double y, bool skipPop){
    // Rechtes Mikrofon: Integral abndern und Magnitude berechnen

    double sinRightPush = sinLUT.at(sinCounter)*y;
    double cosRightPush = cosLUT.at(cosCounter)*y;
    rightMicroSin.replace(listCounter, sinRightPush);
    rightMicroCos.replace(listCounter, cosRightPush);

    integralSinRight = sinRightPush;
    integralCosRight = cosRightPush;

    if(!skipPop){
        double sinRightPop = rightMicroSin.at(window-1-listCounter);
        double cosRightPop = rightMicroCos.at(window-1-listCounter);

        integralSinRight -= sinRightPop;
        integralCosRight -= cosRightPop;

        return pow(integralSinRight, 2.0)+pow(integralCosRight, 2.0);
    } else {
        return 0.0;
    }
}




void PingerDetection::pingerDetectionCallback(const std_msgs::StringConstPtr& msg){
    QString state = msg->data.c_str();

    if(state.contains("start")){
        ROS_INFO("Pingerdetection callback start");
        if(!enabled){
            this->setEnabled(true);
            this->initVariables();
            this->configAudio();
            this->periodicSinCos();
            this->start();
        }
    } else if(state.contains("stop")){
        ROS_INFO("Pingerdetection callback enable");
        this->setEnabled(false);
    } else {
        ROS_INFO("Pingerdetection callback parameter %s is not valid", msg->data.c_str());
    }

}

void PingerDetection::dynReconfigureCallback(hanse_pingerdetection::PingerdetectionNodeConfig &config, uint32_t level) {

    ROS_INFO("Dynamic reconfigure pingerdetection");

    publishTimer.setPeriod(ros::Duration(1.0/config.publish_frequency));

    omega = config.omega;
    sampleRate = config.sampleRate;
    window = config.window;
    scale = config.scale;
    threshold = config.threshold;
    noiseLeft = config.noiseLeft;
    noiseRight = config.noiseRight;

    int samplesPPRaw = config.samplesPerPixelRaw;
    audioPlotRaw.setSamplesPerPixel(samplesPPRaw);
    int samplesPPGoertzel = config.samplesPerPixelGoertzel;
    audioPlotGoertzel.setSamplesPerPixel(samplesPPGoertzel);
    int counterRaw = config.counterRaw;
    audioPlotRaw.setCounter(counterRaw);
    int counterGoertzel = config.counterGoertzel;
    audioPlotGoertzel.setCounter(counterGoertzel);

    plotRaw = config.plotRaw;
    plotGoertzel = config.plotGoertzel;
    saveData = config.saveData;
    plotAnalysis = config.plotAnalysis;

    lognr = config.lognr;
    //ROS_INFO("omega %f", omega);
    //ROS_INFO("sampleRate %i", sampleRate);
    //ROS_INFO("window %i", window);
    //ROS_INFO("scale %f", scale);
    //ROS_INFO("threshold %f", threshold);
    //ROS_INFO("noiseLeft %i", noiseLeft);
    //ROS_INFO("noiseRight %i", noiseRight);
    //ROS_INFO("samplesPerPixelRaw %i", samplesPPRaw);
    //ROS_INFO("samplesPerPixelGoertzel %i", samplesPPGoertzel);
    //ROS_INFO("counterRaw %i", counterRaw);
    //ROS_INFO("counterGoertzel %i", counterGoertzel);

    fetteSkalierung = config.fetteSkalierung;

    ROS_INFO("plotRaw %d", plotRaw);
    ROS_INFO("plotGoertzel %d", plotGoertzel);
    ROS_INFO("saveData %d", saveData);
}

PingerDetection::PingerDetection() :
    audioPlotRaw(nh, 640, 480, "/pingerdetection/plotRaw"),
    audioPlotGoertzel(nh, 640, 480, "/pingerdetection/plotGoertzel")
{
    ROS_INFO("Starting PingerDetection");

    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, ".");
    QSettings::setDefaultFormat(QSettings::IniFormat);

    input_subscriber = nh.subscribe("/hanse/pingerdetection",1, &PingerDetection::pingerDetectionCallback, this);
    angle_publisher = nh.advertise<std_msgs::Float32>("/hanse/pingerdetection/angle", 100);
    pinger_publisher = nh.advertise<hanse_msgs::PingerDetection>("/hanse/pinger", 10);

    //    dynReconfigureCb = boost::bind(&PingerDetection::dynReconfigureCallback, this, _1, _2);
    //    dynReconfigureSrv.setCallback(dynReconfigureCb);
    // from this point on we can assume a valid config

    enabled = false;
}

PingerDetection::~PingerDetection() {
    std::cout << "Destroying PingerDetection." << std::endl;
}

void PingerDetection::setEnabled(bool b) {
    if (b) {
        ROS_INFO("Enabling pingerdetection");
        enabled = true;
    } else {
        ROS_INFO("Disabling pingerdetection");

        enabled = false;
    }
}


}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "PingerDetection");

    ros::start();

    hanse_pingerdetection::PingerDetection pinger_detection;
    //pinger_detection.start();

    // will be set to actual value once config is loaded
    dynamic_reconfigure::Server<hanse_pingerdetection::PingerdetectionNodeConfig> server;
    dynamic_reconfigure::Server<hanse_pingerdetection::PingerdetectionNodeConfig>::CallbackType f;

    f = boost::bind(&hanse_pingerdetection::PingerDetection::dynReconfigureCallback, &pinger_detection, _1, _2);
    server.setCallback(f);

    QApplication app(argc, argv);
    //app.exec();

    ros::spin();

    return 0;
}
