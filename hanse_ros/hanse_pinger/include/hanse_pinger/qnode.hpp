/**
 * @file /include/hanse_pinger/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hanse_pinger_QNODE_HPP_
#define hanse_pinger_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMainWindow>
#include <QtCore>
#include <QtMultimediaKit/QAudio>
#include <QtMultimediaKit/QAudioFormat>
#include <QtMultimediaKit/QAudioInput>
#include <QtMultimediaKit/QAudioDeviceInfo>
#include <QIODevice>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hanse_pinger {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, QString message);


signals:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    ros::Publisher angle_publisher;
    ros::Publisher msg_publisher;
    QStringListModel logging_model;

    int noiseLeft;
    int noiseRight;


    void calculateAngle(int leftMicroSampleDelay, double leftMicroMagnitude, int rightMicroSampleDelay, double rightMicroMagnitude);
    double getAngle();
    double angle;



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

};

}  // namespace hanse_pinger

#endif /* hanse_pinger_QNODE_HPP_ */
