#include "pingerdetection.h"
#include <QApplication>
namespace hanse_pingerdetection{

void PingerDetection::run()
{
    QEventLoop eventLoop;
    eventLoop.processEvents();
    setEnabled(true);

    outputFile = new QFile("pingerdetection.txt");
    outputFile->remove();

    if (outputFile->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream = new QTextStream(outputFile);
        ROS_INFO("Created file %s", outputFile->fileName().toStdString().c_str());
    } else {
        ROS_INFO("Could not open file to write %s", outputFile->fileName().toStdString().c_str());
    }

    outputFileFSK = new QFile("pingerdetectionFSK.txt");
    outputFileFSK->remove();

    if (outputFileFSK->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStreamFSK = new QTextStream(outputFileFSK);
        ROS_INFO("Created file %s", outputFileFSK->fileName().toStdString().c_str());
    } else {
        ROS_INFO("Could not open file to write %s", outputFileFSK->fileName().toStdString().c_str());
    }

    ioDevice = audioInput->start();

    // Komische Ausschlaege am Anfang einer Messung uebergehen
    while(sampleCounter < 960000){
        qint64 bytesReady = audioInput->bytesReady();
        qint64 l = ioDevice->read(audioBuffer.data(), bytesReady);
        Q_UNUSED(l)
        sampleCounter++;
    }
    ROS_INFO("Skipped first samples");
    sampleCounter = 0;

    QTime timestamp;
    timestamp.restart();

    while (ros::ok() && enabled) {

        if(!audioInput){
            ROS_ERROR("No audio input");
            return;
        }

        // Bereitstehende Daten ermitteln und anschließend ggf. lesen
        qint64 bytesReady = audioInput->bytesReady();
        if(bytesReady > sampleRate){
            bytesReady = sampleRate;
        }

        QByteArray newData = ioDevice->readAll();
        qint64 l = newData.length();

        if(l > 0) {
            // ROS_INFO("Samples %i", sampleCounter);

            //            ROS_INFO("Bytes read %s", QString::number(l).toStdString().c_str());

            QList<int> decodeList;
            // Neue Daten von der Soundkarte decodieren
            decodeList.append(this->decodeData(newData.constData(),newData.length()));

            if(!decodeList.isEmpty()){

                //                ROS_INFO("Decoded bytes %i", decodeList.length());

                double zerolineLeft = 0.0;
                double zerolineRight = 0.0;

                for(int i = 0; i < decodeList.length()-2; i+=2){

                    // Decodierte Daten von Rauschen befreien...
                    zerolineLeft = ((double)decodeList.at(i)-(double)noiseLeft)*scale;
                    zerolineRight = ((double)decodeList.at(i+1)-(double)noiseRight)*scale;

                    std_msgs::Float32 leftMessage;
                    leftMessage.data = zerolineLeft;
                    left_publisher.publish(leftMessage);

                    std_msgs::Float32 rightMessage;
                    rightMessage.data = zerolineRight;
                    right_publisher.publish(rightMessage);

                    // ... und in Listen zur Verarbeitung speichern
                    leftMicro.append(zerolineLeft);
                    rightMicro.append(zerolineRight);


                    saveListTime.append(timestamp.elapsed());
                    saveListLeftMicroInt.append(leftMicro.last());
                    saveListRightMicroInt.append(rightMicro.last());
                    // Ermittelte Daten verarbeiten
                    this->audioProcessing();
                }
                //                ROS_INFO("Left micro %f - Right micro %f", zerolineLeft, zerolineRight);

            }
        }
        ros::spinOnce();
    }

    ROS_INFO("Samples %i", sampleCounter);


    if(outputFile->isWritable()){
        ROS_INFO("Write raw data...");
        for(int i = 0; i < saveListTime.length(); i++){
            *outputStream << saveListTime.at(i) << ", " << saveListLeftMicroInt.at(i)  << ", " << saveListRightMicroInt.at(i) << "\r\n";
            outputStream->flush();
        }
        ROS_INFO("Finished raw data writing");
    }

    if(outputFileFSK->isWritable()){
        ROS_INFO("Write FSK data...");

        for(int i = 0; i < saveListTime.length(); i++){
            *outputStreamFSK << saveListTime.at(i)  << ", " << saveListLeftFSK.at(i)  << ", " << saveListRightFSK.at(i) << ", " << saveListAngle.at(i) << "\r\n";
            outputStreamFSK->flush();
        }
        ROS_INFO("Finished FSK data writing");
    }

    saveListTime.clear();
    saveListLeftFSK.clear();
    saveListRightFSK.clear();
    saveListLeftMicroInt.clear();
    saveListRightMicroInt.clear();
    saveListAngle.clear();

    ROS_INFO("Finished");
    audioInput->stop();

}

void PingerDetection::initVariables(){
    ROS_INFO("Init pinger detection variables");

    sampleCounter = 0;

    integralSinLeft = integralCosLeft = integralSinRight = integralCosRight = 0.0;

    delayAverageElements = 0;
    leftAverageThreshold = rightAverageThreshold = 0.0;

    bothMicroSignalDetectedCount = 0;
    leftMicroSampleDelay = rightMicroSampleDelay = 0;
    leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

    delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
    cosCounter = sinCounter = 0;
    leftFSK = rightFSK = 0.0;

    leftMicroSignalDetected = rightMicroSignalDetected = 0;

    angle = 0.0;

    noiseLeft = noiseRight = 0;

    leftMicro.clear();
    rightMicro.clear();

    leftMicroSin.clear();
    leftMicroCos.clear();
    rightMicroSin.clear();
    rightMicroCos.clear();

    QSettings* settings = new QSettings(QSettings::IniFormat, QSettings::UserScope, "ini", "qtSettings");

    settings->beginGroup("PingerDetection");
    omega = settings->value("Omega", 15000).toInt();
    ROS_INFO("Omaga %f",omega);
    sampleRate = settings->value("SampleRate", 48000).toInt();
    ROS_INFO("Sample rate %i",sampleRate);
    window = settings->value("Window", 480).toInt();
    ROS_INFO("Window %i",window);
    scale = settings->value("Scale", 0.00001).toDouble();
    ROS_INFO("Scaling %f",scale);
    threshold = settings->value("Threshold", 0.5).toDouble();
    ROS_INFO("Threshold %f",threshold);
    noiseLeft = settings->value("NoiseLeft", 0).toInt();
    ROS_INFO("Noise left %d",noiseLeft);
    noiseRight = settings->value("NoiseRight", 0).toInt();
    ROS_INFO("Noise right %d",noiseRight);
    settings->endGroup();

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
    // Wilde decodierung des PCM Signals

    QList<int> decodeList;

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

            decodeList.append(value);

            ptr += channelBytes;
        }


    }
    return decodeList;
}

void PingerDetection::audioProcessing(){
    ROS_DEBUG("Audio processing");

    if(leftMicro.length() > window && rightMicro.length() > window){
        // Gewnschte Fensterlnge wurde erreicht

        // Daten verarbeiten
        leftFSK =  leftNonCoherentFSKDemodulator(leftMicro.last(), false);
        rightFSK =  rightNonCoherentFSKDemodulator(rightMicro.last(), false);

        saveListLeftFSK.append(leftFSK);
        saveListRightFSK.append(rightFSK);

        // Sin/Cos Zaehler erhhen
        inkrementSinCosCounter();

        // FSK Werte verwursten
        signalDelayAnalysis(leftFSK, rightFSK);

        saveListAngle.append(angle);

        // Erste Eintrge loeschen, da wir gleiten :)
        leftMicro.removeFirst();
        rightMicro.removeFirst();

    } else {
        // Integral schrittweise bilden
        leftNonCoherentFSKDemodulator(leftMicro.last(), true);
        rightNonCoherentFSKDemodulator(rightMicro.last(), true);
        inkrementSinCosCounter();
        saveListLeftFSK.append(0.0);
        saveListRightFSK.append(0.0);
        saveListAngle.append(0.0);
    }

    sampleCounter++;
}

void PingerDetection::signalDelayAnalysis(double left, double right){

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

    if(delayState == STATE_WAIT_FOR_FIRST_SIGNAL){

        signalDetected = leftMicroSignalDetected + rightMicroSignalDetected;

        if(signalDetected == 2){
            // Beide Mikros haben ein Signal grer als den Schwellwert detektiert
            // Keine messbare Zeitverzgerung

            bothMicroSignalDetectedCount++;

            if(left > leftMicroMaxMagnitude){
                leftMicroMaxMagnitude = left;
            }
            if(right > rightMicroMaxMagnitude){
                rightMicroMaxMagnitude = right;
            }

            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else if(signalDetected == 1){

            if(leftMicroSignalDetected == 1){
                // Linkes Mikro hat ein Signal grer als den Schwellwert
                delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

                if(left > leftMicroMaxMagnitude){
                    leftMicroMaxMagnitude = left;
                }
                // Rechtes Mikro hat ein Delay
                rightMicroSampleDelay++;

            } else if(rightMicroSignalDetected == 1){
                // Rechtes Mikro hat ein Signal grer als den Schwellwert
                delayState = STATE_WAIT_FOR_LEFT_MICRO_SIGNAL;

                if(right > rightMicroMaxMagnitude){
                    rightMicroMaxMagnitude = right;
                }
                // Linkes Mikro hat ein Delay
                leftMicroSampleDelay++;

            } else {
                //ROS_INFO("State error");
            }
        }
    } else if(delayState == STATE_WAIT_FOR_LEFT_MICRO_SIGNAL){

        if(left > threshold){
            leftMicroSignalDetected = 1;

            if(left > leftMicroMaxMagnitude){
                leftMicroMaxMagnitude = left;
            }

            bothMicroSignalDetectedCount++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            leftMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            leftMicroSampleDelay++;
        }

        if(right > rightMicroMaxMagnitude){
            rightMicroMaxMagnitude = right;
        }

    } else if(delayState == STATE_WAIT_FOR_RIGHT_MICRO_SIGNAL){

        if(right > threshold){
            rightMicroSignalDetected = 1;

            if(right > rightMicroMaxMagnitude){
                rightMicroMaxMagnitude = right;
            }

            bothMicroSignalDetectedCount++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            rightMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            rightMicroSampleDelay++;
        }

        if(left > leftMicroMaxMagnitude){
            leftMicroMaxMagnitude = left;
        }



    } else if(delayState == STATE_WAIT_FOR_SIGNAL_FINISHED){

        // Bisschen Toleranz um leichte Schwingungen nicht ins Gewicht fallen zu lassen.
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

        if(left > leftMicroMaxMagnitude){
            leftMicroMaxMagnitude = left;
        }

        if(right > rightMicroMaxMagnitude){
            rightMicroMaxMagnitude = right;
        }

        if(signalDetected == 2){

            bothMicroSignalDetectedCount++;

        } else if(signalDetected == 0){

            //////////////////////////////////////////////////////////////////////////////////////////////////
            // leftMircoSampleDelay > rightMicroSampleDelay => Der Pinger befindet sich rechts
            //      => calculateAngle erhält einen positiven Wert
            // leftMircoSampleDelay < rightMicroSampleDelay => Der Pinger befindet sich links
            //      => calculateAngle erhält einen negativen Wert
            //////////////////////////////////////////////////////////////////////////////////////////////////

            if(leftMicroSampleDelay > rightMicroSampleDelay){
                ROS_INFO("Verzögerungswert [%i]", leftMicroSampleDelay);
                calculateAngle(leftMicroSampleDelay);
            }else if (leftMicroSampleDelay < rightMicroSampleDelay){
                ROS_INFO("Verzögerungswert [%i]", rightMicroSampleDelay*(-1));
                calculateAngle(rightMicroSampleDelay*-1);
            }else if (leftMicroSampleDelay == rightMicroSampleDelay){
                calculateAngle(0);
            }


            delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
            bothMicroSignalDetectedCount = 0;
            leftMicroSampleDelay = rightMicroSampleDelay = 0;
            leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;
        }


    } else {
        //ROS_INFO("Pinger detection state error");
    }
}

void PingerDetection::calculateAngle(int samplediff){

    std_msgs::Float32 winkel;

    ROS_DEBUG("Calcualte angle");

    if(samplediff == 0){
        winkel.data = 0.0;
    }else{

        double baseline = 0.4; // Distanz zwischen den Hydrophonen in m

        double delta_t = 48000/samplediff; // Berechnung des Zeitunterschieds in Sekunden

        double distdiff = 1500 / delta_t; // Berechnung der zurückgelegten Wegstrecke aus dem Zeitunterschied.

        angle = distdiff / baseline;


        winkel.data = angle;
    }

    angle_publisher.publish(winkel);

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

    //  //qDebug() << "sinLUT.length " << sinLUT.length() << " cosLUT.length  " << cosLUT.length();
    //  //qDebug() << "sinLUT.first " << sinLUT.first() << " cosLUT.first  " << cosLUT.first();
    // //qDebug() << "sinLUT.last " << sinLUT.last() << " cosLUT.last  " << cosLUT.last();

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
    leftMicroSin.append(sinLeftPush);
    leftMicroCos.append(cosLeftPush);

    integralSinLeft = sinLeftPush;
    integralCosLeft = cosLeftPush;

    if(!skipPop){
        double sinLeftPop = leftMicroSin.first();
        double cosLeftPop = leftMicroCos.first();

        integralSinLeft -= sinLeftPop;
        integralCosLeft -= cosLeftPop;

        leftMicroSin.removeFirst();
        leftMicroCos.removeFirst();

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
    rightMicroSin.append(sinRightPush);
    rightMicroCos.append(cosRightPush);

    integralSinRight = sinRightPush;
    integralCosRight = cosRightPush;

    if(!skipPop){
        double sinRightPop = rightMicroSin.first();
        double cosRightPop = rightMicroCos.first();

        integralSinRight -= sinRightPop;
        integralCosRight -= cosRightPop;

        rightMicroSin.removeFirst();
        rightMicroCos.removeFirst();

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

            this->start();
        }
    } else if(state.contains("stop")){
        ROS_INFO("Pingerdetection callback enable");
        this->setEnabled(false);
    } else {
        ROS_INFO("Pingerdetection callback parameter %s is not valid", msg->data.c_str());
    }

}

PingerDetection::PingerDetection() {
    ROS_INFO("Starting PingerDetection");

    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, ".");
    QSettings::setDefaultFormat(QSettings::IniFormat);

    // Auf Reihenfolge achten!
    this->initVariables();
    this->periodicSinCos();
    this->configAudio();

    input_subscriber = nh.subscribe("/hanse/pingerdetection",1, &PingerDetection::pingerDetectionCallback, this);
    angle_publisher = nh.advertise<std_msgs::Float32>("/hanse/pingerdetection/angle", 100);
    left_publisher = nh.advertise<std_msgs::Float32>("/hanse/pingerdetection/left_raw", 10);
    right_publisher = nh.advertise<std_msgs::Float32>("/hanse/pingerdetection/right_raw", 10);

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

    QApplication app(argc, argv);

    ros::spin();

    return 0;
}
