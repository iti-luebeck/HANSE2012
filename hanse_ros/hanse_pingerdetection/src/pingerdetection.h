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

#include "hanse_pingerdetection/PingerdetectionNodeConfig.h"
#include <dynamic_reconfigure/server.h>

#include "audioplot.h"

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
    void dynReconfigureCallback(hanse_pingerdetection::PingerdetectionNodeConfig &config, uint32_t level);

public slots:
    void receiveAudio();

private:
    // Methods

    void setEnabled(bool b);

    void setRecordSource(QString n);

    void pingerDetectionCallback(const std_msgs::StringConstPtr &msg);

    void initVariables();
    void configAudio();

    void audioProcessing(double leftRaw, double rightRaw);

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
    ros::Publisher left_publisher;
    ros::Publisher right_publisher;

    //Dyn Reconfigure

    ros::Timer publishTimer;

//    hanse_pingerdetection::PingerdetectionNodeConfig config;

    /** \brief dynamic_reconfigure interface */
//    dynamic_reconfigure::Server<hanse_pingerdetection::PingerdetectionNodeConfig> dynReconfigureSrv;

    /** \brief dynamic_reconfigure call back */
//    dynamic_reconfigure::Server<hanse_pingerdetection::PingerdetectionNodeConfig>::CallbackType dynReconfigureCb;

    //End of dyn reconfigure stuff

    // Plot

    AudioPlot audioPlotRaw;
        AudioPlot audioPlotGoertzel;

    bool detection;

    bool plotRaw;
    bool plotGoertzel;
    bool plotAngle;
    bool saveData;

    // Variables
    int lognr;

    bool enabled;

    int noiseLeft;
    int noiseRight;
    double angle;

    int timer;

        std_msgs::Float32 winkel;


        double leftMicroAverageMagnitude;
        int leftMicroAvgCounter;
        double rightMicroAverageMagnitude;
        int rightMicroAvgCounter;

    QAudioInput *audioInput;
    QIODevice *ioDevice;

    QFile *outputFile;
    QTextStream* outputStream;

    QFile *outputFileFSK;
    QTextStream* outputStreamFSK;


    QFile *inputFile;
    QTextStream* inputStream;

    QString recordSource;

    QByteArray audioBuffer;
    QAudioFormat audioFormat;

    QList<int> decodeData(const char *data, qint64 len);

    quint16 audioMaxAmplitude;

    int sampleCounter;

    int listCounter;

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

    void leftMicroMagnitudeCalculation(double left);
    void rightMicroMagnitudeCalculation(double right);


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
