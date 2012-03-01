#ifndef HANSE_ECHOSOUNDERDRIVER_H
#define HANSE_ECHOSOUNDERDRIVER_H


#include "hanse_msgs/EchoSounder.h"
#include "hanse_sonardriver/EchoSounderConfig.h"
#include "echosounderswitchcommand.h"

#include "sonardriver.h"

class EchoSounderDriver : public SonarDriver<
    hanse_sonardriver::EchoSounderConfig,
    hanse_msgs::EchoSounder,
    EchoSounderSwitchCommand>
{
public:
    EchoSounderDriver(ros::NodeHandle handle);

protected:
    void updateSwitchCommand(EchoSounderSwitchCommand &cmd,
                             const hanse_sonardriver::EchoSounderConfig &config);
    void completeMessage(hanse_msgs::EchoSounder &msg, const SonarReturnData &returnData);
};


#endif
