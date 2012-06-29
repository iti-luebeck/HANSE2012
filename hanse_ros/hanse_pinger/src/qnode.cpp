/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <QString>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include "../include/hanse_pinger/qnode.hpp"

#include <QtCore>
#include <QtMultimediaKit/QAudio>
#include <QtMultimediaKit/QAudioFormat>
#include <QtMultimediaKit/QAudioInput>
#include <QtMultimediaKit/QAudioDeviceInfo>
#include <QIODevice>




/*****************************************************************************
** Defines
*****************************************************************************/

#define STATE_WAIT_FOR_FIRST_SIGNAL "Wait for signal"
#define STATE_COUNT_DELAY "Count delay between both micros"
#define STATE_WAIT_FOR_SIGNAL_FINISHED "Wait until no signal is detected"
#define STATE_SAVE_DATA "Save data"

#define STATE_WAIT_FOR_LEFT_MICRO_SIGNAL "Wait left micro signal"
#define STATE_WAIT_FOR_RIGHT_MICRO_SIGNAL "Wait right micro signal"


#define STATE_BOTH_MICRO_SIGNAL_DETECTED "Both micros got a signal at the same time"
#define STATE_LEFT_MICRO_SIGNAL_DETECTED "Left micro got a signal first"
#define STATE_RIGHT_MICRO_SIGNAL_DETECTED "Right micro got a signal first"


#define RUNMODE_READ_FROM_FILE "Read data from file"
#define RUNMODE_RUNTIME "Receive data at runtime"


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace QtMultimediaKit;
namespace hanse_pinger {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"hanse_pinger");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    angle_publisher = n.advertise<std_msgs::Float32>("/hanse/pinger", 1000);
    msg_publisher = n.advertise<std_msgs::String>("/hanse/pinger_msgs", 1000);

    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"hanse_pinger");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    angle_publisher = n.advertise<std_msgs::String>("/hanse/pinger", 1000);
    msg_publisher = n.advertise<std_msgs::String>("/hanse/pinger_msgs", 1000);

    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(1);
    int count = 0;
    std_msgs::String msg;

    // Auf Reihenfolge achten!
    this->initVariables();
    this->periodicSinCos();
    this->configAudio();

    if(runMode == RUNMODE_READ_FROM_FILE){

        this->log(Info, "Einlesen gestartet");

        while (ros::ok()  && !inputStream->atEnd()) {
            QString line = inputStream->readLine();
            QStringList tempList = line.split(",", QString::SkipEmptyParts);

            // Auf Spaltennummern achten!
            leftMicro.append(tempList.at(0).toDouble());
            rightMicro.append(tempList.at(2).toDouble());

            this->audioProcessing();

            this->getAngle();
        }

        this->log(Info, "Einlesen beendet");

        this->saveData();

    } else if(runMode == RUNMODE_RUNTIME){

        this->log(Info, "Frequency "+QString::number(audioFormat.frequency()));
        this->log(Info, "Channels "+QString::number(audioFormat.channels()));
        this->log(Info, "Sample size  "+QString::number(audioFormat.sampleSize()));
        this->log(Info, "Byte order enum (Endian) "+QString::number(audioFormat.byteOrder()));
        this->log(Info, "Sample type enum "+QString::number(audioFormat.sampleType()));


        audioInput = new QAudioInput(audioFormat, this);
        ioDevice = audioInput->start();

        // Komische Ausschlge am Anfang einer Messung bergehen
        timer.restart();
        while(timer.elapsed() < 5000){
            qint64 bytesReady = audioInput->bytesReady();
            if(bytesReady > sampleRate){
                bytesReady = sampleRate;
            }
            qint64 l = ioDevice->read(audioBuffer.data(), bytesReady);
            Q_UNUSED(l)
        }
        timer.restart();


        while (ros::ok() && (timer.elapsed() < recordTime)) {

            if(!audioInput){
                this->log(Fatal, "No audio input");
                return;
            }

            // Bereitstehende Daten ermitteln und anschließend ggf. lesen
            qint64 bytesReady = audioInput->bytesReady();
            if(bytesReady > sampleRate){
                bytesReady = sampleRate;
            }

            qint64 l = ioDevice->read(audioBuffer.data(), bytesReady);

            if(l > 0) {
                // Neue Daten von der Soundkarte decodieren
                QList<int> decodeList = this->decodeData(audioBuffer.constData(), l);

                if(!decodeList.isEmpty()){
                    for(int i = 0; i < decodeList.length()-2; i+=2){
                        // Decodierte Daten von Rauschen befreien...
                        double zerolineLeft = ((double)decodeList.at(i)-(double)noiseLeft)*scale;
                        double zerolineRight = ((double)decodeList.at(i+1)-(double)noiseRight)*scale;

                        // ... und in Listen zur Verarbeitung speichern
                        leftMicro.append(zerolineLeft);
                        rightMicro.append(zerolineRight);

                        this->audioProcessing();
                    }
                }
            }

            this->getAngle();


            std::stringstream ss;
            ss << "hello world " << count;
            msg.data = ss.str();
            angle_publisher.publish(msg);
            //log(Info,std::string("I sent: ")+msg.data);
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }

        audioInput->stop();

        this->saveData();

        this->log(Info, "Messung beendet");

    } else {

        this->log(Error, "Wrong run mode!");

    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::initVariables(){

    sampleCounter = 0;
    recordTime = 0;

    recordLogname = "aufnahme";

    integralSinLeft = integralCosLeft = integralSinRight = integralCosRight = 0.0;

    delayAverageElements = 0;
    leftAverageThreshold = rightAverageThreshold = 0.0;

    bothMicroSignalDetectedCount = 0;
    leftMicroSampleDelay = rightMicroSampleDelay = 0;
    leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

    delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
    runMode = RUNMODE_RUNTIME;
    cosCounter = sinCounter = 0;
    leftFSK = rightFSK = 0.0;

    leftMicroSignalDetected = rightMicroSignalDetected = 0;

    angle = 0.0;

    noiseLeft = noiseRight = 0;

    leftMicro.clear();
    rightMicro.clear();
    saveListTime.clear();
    saveListLeftMicro.clear();
    saveListLeftFSK.clear();
    saveListRightMicro.clear();
    saveListRightFSK.clear();
    saveListDelay0.clear();
    saveListDelay1.clear();
    saveListDelay2.clear();
    saveListDelay3.clear();
    saveListDelay4.clear();

    leftMicroSin.clear();
    leftMicroCos.clear();
    rightMicroSin.clear();
    rightMicroCos.clear();

    QSettings settings("Qt-Ros Package", "hanse_pinger");
    this->sampleRate = settings.value("SampleRate", 48000).toInt();
    this->window = settings.value("Window", 480).toInt();
    this->scale = settings.value("Scale", 0.00001).toDouble();
    this->threshold = settings.value("Threshold", 0.5).toDouble();
    this->runMode = settings.value("RunMode", RUNMODE_RUNTIME).toString();
    this->setRecordTime(settings.value("RecordTime", 20).toInt());
    this->setRecordLogname(settings.value("RecordLogname", "testlog01").toString());
    this->setRecordSource(settings.value("RecordSource", "testinput").toString());
    this->noiseLeft = settings.value("NoiseLeft", 0).toInt();
    this->noiseRight = settings.value("NoiseRight", 0).toInt();


    this->log(Info, "Pinger Settings:");
    this->log(Info, "Sample rate "+QString::number(this->sampleRate));
    this->log(Info, "Window "+QString::number(this->window));
    this->log(Info, "Scale "+QString::number(this->scale));
    this->log(Info, "Threshold "+QString::number(this->threshold));
    this->log(Info, "Run mode "+this->runMode);
}

void QNode::configAudio(){
    audioBuffer.resize(sampleRate);
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

    QAudioDeviceInfo info;
    info = QAudioDeviceInfo::defaultInputDevice();

    QList<int> supSR = info.supportedSampleRates();
    this->log(Info, "Supported sample rates");

    for(int i = 0; i < supSR.length(); i++){
        this->log(Info, QString::number(supSR.at(i)));
    }

    QList<QAudioDeviceInfo> list = info.availableDevices(QAudio::AudioInput);
    for(int i = 0; i < list.length(); i++){
        QString deviceTemp = list.at(i).deviceName();
        this->log(Info, "Device "+QString::number(i)+" : "+deviceTemp);
        if(deviceTemp.contains("Eingang")){
            info = list.at(i);
        }
    }

    if(!info.isFormatSupported(audioFormat)) {
        this->log(Error, "Format not supported - try to use nearest");
        audioFormat = info.nearestFormat(audioFormat);
    }

    this->log(Info, "Device name "+info.deviceName());
    QStringList supportedCodecs = info.supportedCodecs();
     this->log(Info, "Supported codecs: ");
    for(int i = 0; i < supportedCodecs.length(); i++){
        this->log(Info, supportedCodecs.at(i));
    }

}

void QNode::setRecordTime(int t){
    // t in Sekunden
    this->recordTime = t*1000;
    this->log(Info, "RecordTime"+QString::number(recordTime));
}

void QNode::setRecordLogname(QString n){
    this->recordLogname = n;

    outputFile = new QFile("neueDaten/"+recordLogname+".goertzel");
    outputFile->remove();

    if (outputFile->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream = new QTextStream(outputFile);
    } else {
        this->log(Fatal, "Could not open file to write"+outputFile->fileName());
    }

    outputFile01 = new QFile("neueDaten/"+recordLogname+"_analysis.goertzel");
    outputFile01->remove();

    if (outputFile01->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream01 = new QTextStream(outputFile01);
    } else {
        this->log(Fatal, "Could not open file to write"+outputFile01->fileName());
    }

    this->log(Info, "Record logname"+recordLogname);
}

void QNode::setRecordSource(QString n){
    this->recordSource = n;
    inputFile = new QFile("neueDaten/"+recordSource+".goertzel");

    if (inputFile->open(QFile::ReadOnly | QIODevice::ReadOnly)) {
        inputStream = new QTextStream(inputFile);
    } else {
        this->log(Fatal, "Could not open file to read"+inputFile->fileName());
    }
}


void QNode::saveData(){

    this->log(Info, "Samples "+QString::number(sampleCounter));
    this->log(Info, "Save data...");

    for(int i = 0; i < sampleCounter; i++){
        *outputStream << saveListLeftMicro.at(i)  << ", " << saveListLeftFSK.at(i) << ", " << saveListRightMicro.at(i) << ", " << saveListRightFSK.at(i) << ", " << saveListDelay0.at(i)  << ", "  << saveListDelay1.at(i) << ", "  << saveListDelay2.at(i)<< ", "  << saveListDelay3.at(i)<< ", "  << saveListDelay4.at(i)<< ", "  << saveListDelay5.at(i)<< ", "  << saveListDelay6.at(i) << "\r\n"; //<< ", "  << saveListDelay7.at(i) << "\r\n";
        outputStream->flush();

        //        *outputStream01 <<  saveListDelay0.at(i)  << ", "  << saveListDelay1.at(i) << ", "  << saveListDelay2.at(i)<< ", "  << saveListDelay3.at(i)<< ", "  << saveListDelay4.at(i)<< ", "  << saveListDelay5.at(i)<< ", "  << saveListDelay6.at(i) << "\r\n";
        //        outputStream01->flush();
    }

    saveListTime.clear();
    saveListLeftMicro.clear();
    saveListLeftFSK.clear();
    saveListRightMicro.clear();
    saveListRightFSK.clear();
    saveListDelay0.clear();
    saveListDelay1.clear();
    saveListDelay2.clear();
    saveListDelay3.clear();
    saveListDelay4.clear();
    saveListDelay5.clear();
    saveListDelay6.clear();
    saveListDelay7.clear();

    this->log(Info, "Save finished");

}

QList<int> QNode::decodeData(const char *data, qint64 len){
    // Wilde decodierung des PCM Signals

    QList<int> decodeList;

    Q_ASSERT(audioFormat.sampleSize() % 8 == 0);
    const int channelBytes = audioFormat.sampleSize() / 8;
    const int sampleBytes = audioFormat.channels() * channelBytes;
    Q_ASSERT(len % sampleBytes == 0);
    const int numSamples = len / sampleBytes;

    const unsigned char *ptr = reinterpret_cast<const unsigned char *>(data);

    for (int i = 0; i < numSamples; ++i) {
        for(int j = 0; j < audioFormat.channels(); ++j) {
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

void QNode::audioProcessing(){
    saveListLeftMicro.append(leftMicro.last());
    saveListRightMicro.append(rightMicro.last());

    if(leftMicro.length() > window && rightMicro.length() > window){
        // Gewnschte Fensterlnge wurde erreicht

        // Daten verarbeiten
        leftFSK = this->leftNonCoherentFSKDemodulator(leftMicro.last(), false);
        rightFSK = this->rightNonCoherentFSKDemodulator(rightMicro.last(), false);

        // Daten in einer Liste speichern, um sie nach der Aufnahme in eine Datei zu schreiben
        saveListLeftFSK.append(leftFSK);
        saveListRightFSK.append(rightFSK);

        // Sin/Cos Zaehler erhhen
        this->inkrementSinCosCounter();

        // FSK Werte verwursten
        this->signalDelayAnalysis(leftFSK, rightFSK);

        // Erste Eintrge loeschen, da wir gleiten :)
        leftMicro.removeFirst();
        rightMicro.removeFirst();

    } else {
        // Integral schrittweise bilden
        this->leftNonCoherentFSKDemodulator(leftMicro.last(), true);
        this->rightNonCoherentFSKDemodulator(rightMicro.last(), true);
        this->inkrementSinCosCounter();

        // Daten in einer Liste speichern, um sie nach der Aufnahme in eine Datei zu schreiben
        saveListLeftFSK.append(0.0);
        saveListRightFSK.append(0.0);

        saveListDelay0.append(0);
        saveListDelay1.append(0);
        saveListDelay2.append(0);
        saveListDelay3.append(0);
        saveListDelay4.append(0);
        saveListDelay5.append(0.0);
        saveListDelay6.append(0.0);
        saveListDelay7.append(" - ");
    }

    sampleCounter++;
}

void QNode::signalDelayAnalysis(double left, double right){

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

            bothMicroSignalDetectedCount ++;

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
                rightMicroSampleDelay ++;

            } else if(rightMicroSignalDetected == 1){
                // Rechtes Mikro hat ein Signal grer als den Schwellwert
                delayState = STATE_WAIT_FOR_LEFT_MICRO_SIGNAL;

                if(right > rightMicroMaxMagnitude){
                    rightMicroMaxMagnitude = right;
                }
                // Linkes Mikro hat ein Delay
                leftMicroSampleDelay ++;

            } else {
                this->log(Error, "State error");
            }
        }
    } else if(delayState == STATE_WAIT_FOR_LEFT_MICRO_SIGNAL){

        if(left > threshold){
            leftMicroSignalDetected = 1;

            if(left > leftMicroMaxMagnitude){
                leftMicroMaxMagnitude = left;
            }

            bothMicroSignalDetectedCount ++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            leftMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            leftMicroSampleDelay ++;
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

            bothMicroSignalDetectedCount ++;
            delayState = STATE_WAIT_FOR_SIGNAL_FINISHED;

        } else {
            rightMicroSignalDetected = 0;
            // Linkes Mikro hat ein Delay
            rightMicroSampleDelay ++;
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

            bothMicroSignalDetectedCount ++;

        } else if(signalDetected == 0){

            this->calculateAngle(leftMicroSampleDelay, leftMicroMaxMagnitude, rightMicroSampleDelay, rightMicroMaxMagnitude);

            delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
            bothMicroSignalDetectedCount = 0;
            leftMicroSampleDelay = rightMicroSampleDelay = 0;
            leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

        }
    } else {
        this->log(Error, "State error");
    }

    saveListDelay0.append(leftMicroSignalDetected);
    saveListDelay1.append(rightMicroSignalDetected);
    saveListDelay2.append(signalDetected);
    saveListDelay3.append(leftMicroSampleDelay);
    saveListDelay4.append(rightMicroSampleDelay);
    saveListDelay5.append(leftMicroMaxMagnitude);
    saveListDelay6.append(rightMicroMaxMagnitude);
    saveListDelay7.append(delayState);

}


void QNode::calculateAngle(int leftMicroSampleDelay, double leftMicroMagnitude, int rightMicroSampleDelay, double rightMicroMagnitude){

    int diff = leftMicroSampleDelay-rightMicroSampleDelay;
    diff = 0;
    // 42.23;

    // TODO
    // ...

    angle = 42.23;
}

double QNode::getAngle(){
    return angle;
}

// ---------------------------------- nichtkohrenter FSK-Demodulator

void QNode::periodicSinCos(){

    sinLUT.clear();
    cosLUT.clear();
    sinCounter = 0;
    cosCounter = 0;

    // Periodische LUT erstellen fr 48k Hz Abtastfrequenz.

    // sin(0) = 0 und sin(24) = 0 (fast)
    for(int i = 0; i < 24; i++){
        double sinTemp = sin(2.0*M_PI*(double)omega*(double)i/(double)sampleRate);
        sinLUT.append(sinTemp);
    }

    // cos(0) = 1 und cos(48) = 1
    for(int i = 0; i < 48; i++){
        double cosTemp = cos(2.0*M_PI*omega*(double)i/(double)sampleRate);
        cosLUT.append(cosTemp);
    }

    //  qDebug() << "sinLUT.length " << sinLUT.length() << " cosLUT.length  " << cosLUT.length();
    //  qDebug() << "sinLUT.first " << sinLUT.first() << " cosLUT.first  " << cosLUT.first();
    // qDebug() << "sinLUT.last " << sinLUT.last() << " cosLUT.last  " << cosLUT.last();

    //    for(int x = 0; x < 70;  x++){
    //        qDebug() << "x= "<< x;
    //        double sinLeft = sin(2.0*M_PI*(double)omega*(double)sinCounter/(double)sampleRate);
    //        double cosLeft = cos(2.0*M_PI*(double)omega*(double)cosCounter/(double)sampleRate);
    //        qDebug() << "sinLeft " << sinLeft << "   cosLeft " << cosLeft;
    //        this->inkrementSinCosCounter();
    //    }

}

void QNode::inkrementSinCosCounter(){
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

double QNode::leftNonCoherentFSKDemodulator(double x, bool skipPop){
    // Linkes Mikrofon: Integral abndern und Magnitude berechnen

    double sinLeftPush = sinLUT.at(sinCounter)*x;
    double cosLeftPush = cosLUT.at(cosCounter)*x;
    leftMicroSin.append(sinLeftPush);
    leftMicroCos.append(cosLeftPush);

    integralSinLeft += sinLeftPush;
    integralCosLeft += cosLeftPush;

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


double QNode::rightNonCoherentFSKDemodulator(double y, bool skipPop){
    // Rechtes Mikrofon: Integral abndern und Magnitude berechnen

    double sinRightPush = sinLUT.at(sinCounter)*y;
    double cosRightPush = cosLUT.at(cosCounter)*y;
    rightMicroSin.append(sinRightPush);
    rightMicroCos.append(cosRightPush);

    integralSinRight += sinRightPush;
    integralCosRight += cosRightPush;

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


void QNode::log( const LogLevel &level, QString message) {
    std::stringstream txt;
    std_msgs::String pub_msg;

    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(message.toStdString());
        txt << "[DEBUG] [" << ros::Time::now() << "]: " << message.toStdString();
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(message.toStdString());
        txt << "[INFO] [" << ros::Time::now() << "]: " << message.toStdString();
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(message.toStdString());
        txt << "[INFO] [" << ros::Time::now() << "]: " << message.toStdString();
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(message.toStdString());
        txt << "[ERROR] [" << ros::Time::now() << "]: " << message.toStdString();
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(message.toStdString());
        txt << "[FATAL] [" << ros::Time::now() << "]: " << message.toStdString();
        break;
    }
    }
    pub_msg.data = txt.str();
    msg_publisher.publish(pub_msg);
}

}  // namespace hanse_pinger
