#ifndef XBOX_H
#define XBOX_H

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

class XboxController
{
    private:
        XboxSeriesXControllerESP32_asukiaaa::Core xbox_controller;
        String target_address = "";
        bool is_connected = false;

    public:
        XboxController(String targetDeviceAddress = "");
        void begin() { xbox_controller.begin(); }
        void onLoop() { xbox_controller.onLoop(); }
        bool isConnected() { return xbox_controller.isConnected(); }
        bool isWaitingForFirstNotification() { return xbox_controller.isWaitingForFirstNotification(); }
        void connectXboxController();
        String xboxString();
        String getTargetAddress() { return target_address; }
        bool getIsConnected() { return is_connected; }
        bool Y(){ return xbox_controller.xboxNotif.btnY; }
        bool X(){ return xbox_controller.xboxNotif.btnX; }
        bool B(){ return xbox_controller.xboxNotif.btnB; }
        bool A(){ return xbox_controller.xboxNotif.btnA; }
        bool LB(){ return xbox_controller.xboxNotif.btnLB; }
        bool RB(){ return xbox_controller.xboxNotif.btnRB; }
        bool Select(){ return xbox_controller.xboxNotif.btnSelect; }
        bool Start(){ return xbox_controller.xboxNotif.btnStart; }
        bool Xbox(){ return xbox_controller.xboxNotif.btnXbox; }
        bool Share(){ return xbox_controller.xboxNotif.btnShare; }
        bool LS(){ return xbox_controller.xboxNotif.btnLS; }
        bool RS(){ return xbox_controller.xboxNotif.btnRS; }
        bool Up(){ return xbox_controller.xboxNotif.btnDirUp; }
        bool Right(){ return xbox_controller.xboxNotif.btnDirRight; }
        bool Down(){ return xbox_controller.xboxNotif.btnDirDown; }
        bool Left(){ return xbox_controller.xboxNotif.btnDirLeft; }
        int LH(){ return xbox_controller.xboxNotif.joyLHori; }
        int LV(){ return xbox_controller.xboxNotif.joyLVert; }
        int RH(){ return xbox_controller.xboxNotif.joyRHori; }
        int RV(){ return xbox_controller.xboxNotif.joyRVert; }
        int LT(){ return xbox_controller.xboxNotif.trigLT; }
        int RT(){ return xbox_controller.xboxNotif.trigRT; }
};

#endif