package org.firstinspires.ftc.teamcode.backend.commandbased;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

import java.util.List;

public abstract class CommandbasedOpmode extends OpMode {

    protected CommandScheduler scheduler = CommandScheduler.getInstance();
    protected ElapsedTime timer = new ElapsedTime();
    protected Robot19397 robot = new Robot19397(timer);
    private List<LynxModule> allHubs;

    public GamepadWrapper pad1;
    public GamepadWrapper pad2;

    @Override
    public void internalPreInit() {
        super.internalPreInit();
        scheduler.beginOpmode();
        pad1 = new GamepadWrapper(gamepad1);
        pad2 = new GamepadWrapper(gamepad2);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    @Override
    public void internalPostLoop() {
        super.internalPostLoop();
        scheduler.update(telemetry);
        robot.update();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

}
