#ifndef HANSE_SCANNINGSONARDRIVER_H
#define HANSE_SCANNINGSONARDRIVER_H


#include "hanse_msgs/ScanningSonar.h"
#include "hanse_sonardriver/ScanningSonarConfig.h"
#include "scanningsonarswitchcommand.h"

#include "sonardriver.h"

class ScanningSonarDriver : public SonarDriver<
    hanse_sonardriver::ScanningSonarConfig,
    hanse_msgs::ScanningSonar,
    ScanningSonarSwitchCommand>
{
public:
    ScanningSonarDriver(ros::NodeHandle handle);

protected:
    void updateSwitchCommand(ScanningSonarSwitchCommand &cmd,
                             const hanse_sonardriver::ScanningSonarConfig &config);
    void completeMessage(hanse_msgs::ScanningSonar &msg, const SonarReturnData &returnData);
};


#endif
