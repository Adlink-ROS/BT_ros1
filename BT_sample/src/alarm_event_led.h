#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "linux/EApiOs.h"
#include "EApi.h"
#include "semaeapi.h"

#define OUT_DIRECTION 0 // out
#define LEVEL_LOW     0
#define LEVEL_HIGH    1

class AlarmEventLED : public BT::AsyncActionNode
{
    public:
        AlarmEventLED(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
            // init SEMA
            char addr[16] = "127.0.0.1";
            int ret = SemaEApiLibInitialize(false, IP_V4, addr, 0, (char *)"123", &libHandle_);
            if (ret == EAPI_STATUS_SUCCESS) {
                printf("[ERROR] NeuronGpio - Can't initialize SEMA Lib. Error code: %X\n", ret);
            }
            // init GPIO
            _pin = 7;
            // Set direction
            ret = SemaEApiGPIOSetDirection(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, OUT_DIRECTION);
            if (ret != EAPI_STATUS_SUCCESS) {
                printf("Error setting GPIO direction: 0x%X\n", ret);
            }
            // Set level low
            ret = SemaEApiGPIOSetLevel(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, LEVEL_LOW);
            if (ret != EAPI_STATUS_SUCCESS) {
                printf("(Error setting GPIO level: 0x%X)\n", ret);
            }
        }

        ~AlarmEventLED()
        {
           SemaEApiLibUnInitialize(libHandle_);
        }

        static BT::PortsList providedPorts()
        {
            return{};
        }

        virtual BT::NodeStatus tick() override
        {
            int ret; 
            // Set level high
            ret = SemaEApiGPIOSetLevel(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, LEVEL_HIGH);
            if (ret != EAPI_STATUS_SUCCESS) {
                printf("(Error setting GPIO level: 0x%X)\n", ret);
            }
            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
        uint32_t libHandle_;
        uint32_t _pin;
};
