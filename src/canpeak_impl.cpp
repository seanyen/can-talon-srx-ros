#include <ros/ros.h>

#include <memory>

#include <windows.h>
#include "PCAN/PCANBasic.h"

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/canpeak.h"

extern std::shared_ptr<can_talon_srx::CanInterface> can_interface;

namespace can_talon_srx
{

static std::runtime_error getException(TPCANStatus status)
{
    char strMsg[256] = {};
    TPCANStatus result = CAN_GetErrorText(status, 0, strMsg);
    if (result == PCAN_ERROR_OK)
    {
        return std::runtime_error(strMsg);
    }
    else
    {
        return std::runtime_error("unknown error");
    }
}

CanPeakInterface::CanPeakInterface(const char *interface_name)
{
    
    auto messageBox = std::make_shared<MessageBox>();
    receivedMessages_ = messageBox;

    TPCANStatus result = CAN_Initialize(PCAN_PCIBUS1, PCAN_BAUD_1M);
    if (result != PCAN_ERROR_OK)
    {
        throw getException(result);
    }

    readThread = std::thread([](std::shared_ptr<MessageBox> messages, std::atomic<bool> *running) {
        ROS_INFO("thread start");
        while (*running)
        {
            TPCANMsg msgReceived = {0};
            TPCANStatus result = CAN_Read(PCAN_USBBUS1, &msgReceived, NULL);
            if (result == PCAN_ERROR_QRCVEMPTY)
            {
                continue;
            }
            else if (result == PCAN_ERROR_OK)
            {
                switch (msgReceived.MSGTYPE)
                {
                    case PCAN_MESSAGE_STANDARD:
                        {
                            std::lock_guard<std::mutex> guard(messages->lock);
                            auto &ptr = messages->messages[msgReceived.ID];
                            ROS_INFO("packet %08x", msgReceived.ID);
                            auto new_frame = Message{0};
                            new_frame.arbID = msgReceived.ID;
                            new_frame.len = msgReceived.LEN;
                            memcpy(&new_frame.data, msgReceived.DATA, 8);
                            ptr = std::make_shared<Message>(new_frame);
                        }
                        break;
                    case PCAN_MESSAGE_STATUS:
                        ROS_INFO_STREAM("PCAN_MESSAGE_STATUS is not yet handled.");
                        break;
                    case PCAN_MESSAGE_ERRFRAME:
                        ROS_INFO_STREAM("PCAN_MESSAGE_ERRFRAME is not yet handled.");
                        break;
                    default:
                        ROS_ERROR_STREAM("unknown MSGTYPE");
                        break;
                }
            }
            else
            {
                ROS_ERROR_STREAM("unable to send CAN message: " << result);
                continue;
            }
        }
    }, messageBox, &running);
}

CanPeakInterface::~CanPeakInterface()
{
    running = false;
    readThread.join();
}

void CanPeakInterface::Init(const char *interface_name)
{
    if (can_interface)
    {
        ROS_ERROR("trying to initialize an already initialized CAN interface!");
        return;
    }
    can_interface = std::shared_ptr<CanInterface>(new CanPeakInterface(interface_name));
}

void CanPeakInterface::sendMessage(uint32_t arbID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
{
    if (periodMs == 0)
    {
        // do nothing
    }
    else
    {
        TPCANMsg funcRequest = {0};
        funcRequest.ID = arbID;
        funcRequest.MSGTYPE = PCAN_MESSAGE_STANDARD;
        funcRequest.LEN = 8;

        memcpy(funcRequest.DATA, data, dataSize);

        TPCANStatus result = CAN_Write(PCAN_PCIBUS1, &funcRequest);
        if (result != PCAN_ERROR_OK)
        {
            ROS_ERROR_STREAM("unable to send CAN message: " << result);
            *status = 1;
        }
    }
}

void CanPeakInterface::receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
{
    // check the message box to see if a message has been received
    std::lock_guard<std::mutex> guard(receivedMessages_->lock);
    auto& entry = receivedMessages_->messages[*messageID];
    if (entry)
    {
      *status = 0;
      std::shared_ptr<Message> msg;
      entry.swap(msg);

      memcpy(data, msg->data, msg->len);
      *dataSize = msg->len;
    }
    else
    {
      // no entry; set status accordingly
      *status = 1;
    }
}

void CanPeakInterface::openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status)
{
    throw std::runtime_error("not implemented");
}

void CanPeakInterface::closeStreamSession(uint32_t sessionHandle)
{
    throw std::runtime_error("not implemented");
}

void CanPeakInterface::readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status)
{
    throw std::runtime_error("not implemented");
}

void CanPeakInterface::getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status)
{
    throw std::runtime_error("not implemented");
}

} // namespace can_talon_srx
