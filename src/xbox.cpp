#include "xbox.h"

// Ensure the constructor is defined only once
XboxController::XboxController(String targetDeviceAddress)
{
    xbox_controller = XboxSeriesXControllerESP32_asukiaaa::Core(targetDeviceAddress);
    target_address = targetDeviceAddress;
}

void XboxController::connectXboxController()
{
    xbox_controller.begin();
    while(!is_connected)
    {
        xbox_controller.onLoop();
        if (xbox_controller.isConnected())
        {
            if (xbox_controller.isWaitingForFirstNotification())
            {
                Serial.println("waiting for first notification");
            }
            else
            {
                is_connected = true;
            }
        }
        else
        {
            Serial.println("not connected");
            if (xbox_controller.getCountFailedConnection() > 2)
            {
            ESP.restart();
            }
        }
    }
}

String XboxController::xboxString(){
    String str = String(xbox_controller.xboxNotif.btnY) + "," +
               String(xbox_controller.xboxNotif.btnX) + "," +
               String(xbox_controller.xboxNotif.btnB) + "," +
               String(xbox_controller.xboxNotif.btnA) + "," +
               String(xbox_controller.xboxNotif.btnLB) + "," +
               String(xbox_controller.xboxNotif.btnRB) + "," +
               String(xbox_controller.xboxNotif.btnSelect) + "," +
               String(xbox_controller.xboxNotif.btnStart) + "," +
               String(xbox_controller.xboxNotif.btnXbox) + "," +
               String(xbox_controller.xboxNotif.btnShare) + "," +
               String(xbox_controller.xboxNotif.btnLS) + "," +
               String(xbox_controller.xboxNotif.btnRS) + "," +
               String(xbox_controller.xboxNotif.btnDirUp) + "," +
               String(xbox_controller.xboxNotif.btnDirRight) + "," +
               String(xbox_controller.xboxNotif.btnDirDown) + "," +
               String(xbox_controller.xboxNotif.btnDirLeft) + "," +
               String(xbox_controller.xboxNotif.joyLHori) + "," +
               String(xbox_controller.xboxNotif.joyLVert) + "," +
               String(xbox_controller.xboxNotif.joyRHori) + "," +
               String(xbox_controller.xboxNotif.joyRVert) + "," +
               String(xbox_controller.xboxNotif.trigLT) + "," +
               String(xbox_controller.xboxNotif.trigRT) + "\n";
  return str;
}