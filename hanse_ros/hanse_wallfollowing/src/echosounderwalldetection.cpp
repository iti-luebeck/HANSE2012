#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "std_msgs/Float32.h"
#include "hanse_wallfollowing/WallFollowingConfig.h"
#include "hanse_msgs/EchoSounder.h"

ros::Publisher averageDistance_pub;
int count;
float avgDistance;
float avgSig[252];
float fewSigAvg[5][252];
//config
int averageWindow = 3;
int threshold = 30;
float calcFactor = 0.3;

void callback(hanse_wallfollowing::WallFollowingConfig &config, uint32_t level) 
{
ROS_INFO("Reconfigure Request: %d %d %f", 
            config.wd_averageWindow, config.wd_threshold, 
            config.wd_calcFactor);
    averageWindow = config.wd_averageWindow;
    threshold = config.wd_threshold;
    calcFactor = config.wd_calcFactor;
}

void echoSounderCallback(const hanse_msgs::EchoSounder &msg)
{

    // qDebug("Scanning output");
    float dataLength = msg.echoData.size();
    float temp[(int)dataLength];

    for(int i=0; i<(int)dataLength; i++){
        temp[i] = msg.echoData[i];
    }


    // Gewünschte Ausgabeeinheit
   float einheit = dataLength/msg.range;

    // Variable die Angibt, wie viele Datenwerte gleichzeitig angeguckt werden,
    // um über einen Schwellwert zu kommen.
//    averageWindow = getSettingsValue("averageWindow").toInt();

    // Der eben genannte Schwellwert, optimaler Wert muss noch gesucht werden!
//    threshold = getSettingsValue("threshold").toInt();

    // Berechnete Summe von avgTest Datenwerte
    float avgFilter = 0.0;

    float aktMax = 0.0;
    char c;

    // 5 Datenarrays werden für die Distanzberechnung gefüllt
    if(count%5==4){
        for (int i = 0; i < dataLength; i++) {
            c = temp[i];
            if(c < threshold){
                fewSigAvg[0][i] = 0;
            }else{
                fewSigAvg[0][i] = temp[i];
            }
        }
    } else if(count%5==3){
        for (int i = 0; i < dataLength; i++) {
            c = temp[i];
            if(c < threshold){
                fewSigAvg[1][i] = 0;
            }else{
                fewSigAvg[1][i] = temp[i];
            }
        }
    } else if(count%5==2){
        for (int i = 0; i < dataLength; i++) {
            c = temp[i];
            if(c < threshold){
                fewSigAvg[2][i] = 0;
            }else{
                fewSigAvg[2][i] = temp[i];
            }
        }
    } else if(count%5==1){
        for (int i = 0; i < dataLength; i++) {
            c = temp[i];
            if(c < threshold){
                fewSigAvg[3][i] = 0;
            }else{
                fewSigAvg[3][i] = temp[i];
            }
        }
    } else if(count%5==0){
        for (int i = 0; i < dataLength; i++) {
            c = temp[i];
            if(c < threshold){
                fewSigAvg[4][i] = 0;
            }else{
                fewSigAvg[4][i] = temp[i];
            }
        }
    }

    if(count!=0){
        if(count%5==0){
            count = 0;
        }

        avgDistance = 0.0;
        // Summe aus allen 5 Datenarrays, Durchschnitt pro Datenwert
        for(int i = 0; i < 5; i++){
            for (int j = 0; j < dataLength; j++){
                avgSig[j] = avgSig[j]+fewSigAvg[i][j];
            }
        }

        for(int i=0; i<dataLength;i++){
            avgSig[i] = (avgSig[i]/5);
        }

        for(int i=0; i<dataLength;i++){
            if(avgSig[i]<threshold){
                avgSig[i]=0;
            }
        }

        for (int r = 0; r < 252; r++){

            if(aktMax < avgSig[r]){
                aktMax = avgSig[r];
            }
        }

//        float calcFactor = getSettingsValue("calcFactor").toFloat();

        // Prüfen, ob die naechsten X Datenwerte den Schwellwert überschreiten
       for(int x = 0; x < dataLength-averageWindow; x++){

            for(int y = x; y<x+averageWindow-1; y++){
                avgFilter = avgFilter+avgSig[y];
                // qDebug()<<"avgFilter berechnung"<<avgFilter;
            }

            float a = ((calcFactor)*(float)averageWindow * aktMax);

            if(avgFilter>a){
                avgDistance = (x+3)/einheit;
                // Berechnung abgeschlossen, also raus hier!
                //qDebug("ok");
                break;
            } else {
                avgDistance = 0.0;
                avgFilter = 0.0;
            }
        }
    }

//    addData("distance average", avgDistance);
//    addData("count", count);
//    addData("window average", averageWindow);
//    emit newEchoUiData(avgDistance);
//    emit newWallBehaviourData(data, avgDistance);
//    emit dataChanged(this);

    // Zaehler um zu gucken, wie viele Messungen schon verarbeitet wurden
    count++;

    std_msgs::Float32 avgmsg;
    avgmsg.data = avgDistance;
    averageDistance_pub.publish(avgmsg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "echosounderwalldetection");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<hanse_wallfollowing::WallFollowingConfig> server;
    server.setCallback(boost::bind(&callback, _1, _2));

    for(int i = 0; i < 5; i++){
        for (int j = 0; j < 252; j++){
            fewSigAvg[i][j] = 0;
            avgSig[j] = 0;
        }
    }

    averageDistance_pub = nh.advertise<std_msgs::Float32>("/echosounderaveragedistance", 1000);
    ros::Subscriber sub = nh.subscribe("/hanse/sonar/echo", 1, echoSounderCallback);

    ros::spin();

    return 0;
}
