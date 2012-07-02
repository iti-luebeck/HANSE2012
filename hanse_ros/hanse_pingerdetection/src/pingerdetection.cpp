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

using namespace QtMultimediaKit;

class PingerDetection {
private:
    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::Subscriber input_subscriber_;
    ros::Publisher angle_publisher;
    ros::Publisher msg_publisher;

    bool enabled_;

    int noiseLeft;
    int noiseRight;


    void calculateAngle(int leftMicroSampleDelay, double leftMicroMagnitude, int rightMicroSampleDelay, double rightMicroMagnitude);
    double getAngle();
    double angle;


    bool enable(bool b);

    void readSettings();
    void setRecordTime(int t);
    void setRecordLogname(QString n);
    void setRecordSource(QString n);

    void configAudio();
    void startRecording();
    void stopRecording();

    void initVariables();

    void periodicSinCos();


    QAudioInput *audioInput;
    QIODevice *ioDevice;

    QFile *configFile;
    QTextStream* configStream;

    QFile *outputFile;
    QTextStream* outputStream;

    QFile *outputFile01;
    QTextStream* outputStream01;

    QFile *inputFile;
    QTextStream* inputStream;

    QList<int> inputTime;
    QList<double> inputLeftMicroInt;
    QList<double> inputRightMicroInt;


    int recordTime;
    QString recordLogname;
    QString recordSource;

    QByteArray audioBuffer;
    QAudioFormat audioFormat;

    QList<int> decodeData(const char *data, qint64 len);

    QTime timer;

    quint16 audioMaxAmplitude;


    QString runMode;


    QList<double> leftMicro;
    QList<double> rightMicro;

    QList<int> saveListTime;

    QList<double> saveListLeftMicro;
    QList<double> saveListLeftFSK;
    QList<double> saveListRightMicro;
    QList<double> saveListRightFSK;
    QList<int> saveListDelay0;
    QList<int> saveListDelay1;
    QList<int> saveListDelay2;
    QList<int> saveListDelay3;
    QList<int> saveListDelay4;
    QList<double> saveListDelay5;
    QList<double> saveListDelay6;
    QList<QString> saveListDelay7;

    QList<double> leftMicroCos;
    QList<double> leftMicroSin;

    QList<double> rightMicroCos;
    QList<double> rightMicroSin;

    int sampleCounter;

    QString rosInfo;

    void audioProcessing();

    double integralSinLeft;
    double integralCosLeft;

    double integralSinRight;
    double integralCosRight;


    void saveData();

    QList<double> cosLUT;
    QList<double> sinLUT;
    int cosCounter;
    int sinCounter;

    void inkrementSinCosCounter();

    // Goertzel

    double leftNonCoherentFSKDemodulator(double x, bool skipPop);
    double rightNonCoherentFSKDemodulator(double y, bool skipPop);

    double goertzel_normalized_frequency;

    double omega;

    void signalDelayAnalysis(double goertzelLeft, double goertzelRight);
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
    int sampleRate;
    int window;
    double scale;
    double threshold;


public:

    PingerDetection();
    virtual ~PingerDetection();

public slots:
    void receiveAudio();
};



void PingerDetection::initVariables(){

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
     sampleRate = settings.value("SampleRate", 48000).toInt();
     window = settings.value("Window", 480).toInt();
     scale = settings.value("Scale", 0.00001).toDouble();
     threshold = settings.value("Threshold", 0.5).toDouble();
     runMode = settings.value("RunMode", RUNMODE_RUNTIME).toString();
     setRecordTime(settings.value("RecordTime", 20).toInt());
     setRecordLogname(settings.value("RecordLogname", "testlog01").toString());
     setRecordSource(settings.value("RecordSource", "testinput").toString());
     noiseLeft = settings.value("NoiseLeft", 0).toInt();
     noiseRight = settings.value("NoiseRight", 0).toInt();




    //ROS_INFO("Pinger Settings:");
    //ROS_INFO("Sample rate "+QString::number(sampleRate));
    //ROS_INFO("Window "+QString::number(window));
    //ROS_INFO("Scale "+QString::number(scale));
    //ROS_INFO("Threshold "+QString::number(threshold));
    //ROS_INFO("Run mode "+runMode);
}

void PingerDetection::configAudio(){
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
    //ROS_INFO("Supported sample rates");

    for(int i = 0; i < supSR.length(); i++){
        //ROS_INFO(QString::number(supSR.at(i)));
    }

    QList<QAudioDeviceInfo> list = info.availableDevices(QAudio::AudioInput);
    for(int i = 0; i < list.length(); i++){
        QString deviceTemp = list.at(i).deviceName();
        //ROS_INFO("Device "QString::number(i)" : "deviceTemp);
        if(deviceTemp.contains("Eingang")){
            info = list.at(i);
        }
    }

    if(!info.isFormatSupported(audioFormat)) {
          //ROS_INFO("Format not supported - try to use nearest");
        audioFormat = info.nearestFormat(audioFormat);
    }

    //ROS_INFO("Device name "info.deviceName());
    QStringList supportedCodecs = info.supportedCodecs();
    //ROS_INFO("Supported codecs: ");
    for(int i = 0; i < supportedCodecs.length(); i++){
        //ROS_INFO(supportedCodecs.at(i));
    }

}

void PingerDetection::setRecordTime(int t){
    // t in Sekunden
     recordTime = t*1000;
    //ROS_INFO("RecordTime"+QString::number(recordTime));
}


void PingerDetection::setRecordLogname(QString n){
     recordLogname = n;

    outputFile = new QFile("neueDaten/"+recordLogname+".goertzel");
    outputFile->remove();

    if (outputFile->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream = new QTextStream(outputFile);
    } else {
          //ROS_INFO("Could not open file to write"outputFile->fileName());
    }

    outputFile01 = new QFile("neueDaten/"+recordLogname+"_analysis.goertzel");
    outputFile01->remove();

    if (outputFile01->open(QFile::WriteOnly | QIODevice::Append)) {
        outputStream01 = new QTextStream(outputFile01);
    } else {
          //ROS_INFO("Could not open file to write"outputFile01->fileName());
    }

    //ROS_INFO("Record logname"recordLogname);
}

void PingerDetection::setRecordSource(QString n){
     recordSource = n;
    inputFile = new QFile("neueDaten/"+recordSource+".goertzel");

    if (inputFile->open(QFile::ReadOnly | QIODevice::ReadOnly)) {
        inputStream = new QTextStream(inputFile);
    } else {
          //ROS_INFO("Could not open file to read"inputFile->fileName());
    }
}


void PingerDetection::saveData(){

    //ROS_INFO("Samples "QString::number(sampleCounter));
    //ROS_INFO("Save data...");

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

    //ROS_INFO("Save finished");

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

            ptr = channelBytes;
        }


    }
    return decodeList;
}

void PingerDetection::audioProcessing(){
    saveListLeftMicro.append(leftMicro.last());
    saveListRightMicro.append(rightMicro.last());

    if(leftMicro.length() > window && rightMicro.length() > window){
        // Gewnschte Fensterlnge wurde erreicht

        // Daten verarbeiten
        leftFSK =  leftNonCoherentFSKDemodulator(leftMicro.last(), false);
        rightFSK =  rightNonCoherentFSKDemodulator(rightMicro.last(), false);

        // Daten in einer Liste speichern, um sie nach der Aufnahme in eine Datei zu schreiben
        saveListLeftFSK.append(leftFSK);
        saveListRightFSK.append(rightFSK);

        // Sin/Cos Zaehler erhhen
         inkrementSinCosCounter();

        // FSK Werte verwursten
         signalDelayAnalysis(leftFSK, rightFSK);

        // Erste Eintrge loeschen, da wir gleiten :)
        leftMicro.removeFirst();
        rightMicro.removeFirst();

    } else {
        // Integral schrittweise bilden
         leftNonCoherentFSKDemodulator(leftMicro.last(), true);
         rightNonCoherentFSKDemodulator(rightMicro.last(), true);
         inkrementSinCosCounter();

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

             calculateAngle(leftMicroSampleDelay, leftMicroMaxMagnitude, rightMicroSampleDelay, rightMicroMaxMagnitude);

            delayState = STATE_WAIT_FOR_FIRST_SIGNAL;
            bothMicroSignalDetectedCount = 0;
            leftMicroSampleDelay = rightMicroSampleDelay = 0;
            leftMicroMaxMagnitude = rightMicroMaxMagnitude = 0.0;

        }
    } else {
          //ROS_INFO("Pinger detection state error");
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


void PingerDetection::calculateAngle(int leftMicroSampleDelay, double leftMicroMagnitude, int rightMicroSampleDelay, double rightMicroMagnitude){

    int diff = leftMicroSampleDelay-rightMicroSampleDelay;
    diff = 0;
    // 42.23;

    // TODO
    // ...

    angle = 42.23;
}

double PingerDetection::getAngle(){
    return angle;
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

    //    for(int x = 0; x < 70;  x){
    //        qDebug() << "x= "<< x;
    //        double sinLeft = sin(2.0*M_PI*(double)omega*(double)sinCounter/(double)sampleRate);
    //        double cosLeft = cos(2.0*M_PI*(double)omega*(double)cosCounter/(double)sampleRate);
    //        qDebug() << "sinLeft " << sinLeft << "   cosLeft " << cosLeft;
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




PingerDetection::PingerDetection() {
    //ROS_INFO("Starting PingerDetection");

    enabled_ = false;
}

PingerDetection::~PingerDetection() {
    std::cout << "Destroying PingerDetection." << std::endl;
}

bool PingerDetection::enable(bool b) {
    // //ROS_INFO("Service call: pinger_detection");

    if (b) {
        // //ROS_INFO("Enabling pinger_detection");
        enabled_ = true;
    } else {
        // //ROS_INFO("Disabling pinger_detection");

        enabled_ = false;
    }

    // //ROS_INFO("Service leave: hanse_pidcontrol");
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PingerDetection");

    ros::start();

    PingerDetection pinger_detection;

    return 0;
}
