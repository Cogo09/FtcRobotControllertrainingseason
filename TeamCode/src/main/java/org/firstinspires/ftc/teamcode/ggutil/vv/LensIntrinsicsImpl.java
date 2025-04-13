package org.firstinspires.ftc.teamcode.ggutil.vv;

import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.LensIntrinsics;

public class LensIntrinsicsImpl implements LensIntrinsics {
    private Double fx;
    private Double fy;
    private Double cx;
    private Double cy;

    public LensIntrinsicsImpl(Double fx, Double fy, Double cx, Double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    public LensIntrinsicsImpl() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public Double getFx() {
        return fx;
    }

    @Override
    public void setFx(Double fx) {
        this.fx = fx;
    }

    @Override
    public Double getFy() {
        return fy;
    }

    @Override
    public void setFy(Double fy) {
        this.fy = fy;
    }

    @Override
    public Double getCx() {
        return cx;
    }

    @Override
    public void setCx(Double cx) {
        this.cx = cx;
    }

    @Override
    public Double getCy() {
        return cy;
    }

    @Override
    public void setCy(Double cy) {
        this.cy = cy;
    }
}
