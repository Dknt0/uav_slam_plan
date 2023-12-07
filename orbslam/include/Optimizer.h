/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    /**
     * @brief 光束法平差，GlobalBundleAdjustemnt 中调用了这个函数
     * 
     * @param vpKF 关键帧向量
     * @param vpMP 地图点向量
     * @param nIterations 迭代次数
     * @param pbStopFlag 停止标志位
     * @param nLoopKF ?
     * @param bRobust ?
    */
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
                            
    /**
     * @brief 全局 BA
     * 
     * @param pMap 地图指针
     * @param nIterations 迭代次数
     * @param pbStopFlag 停止标志位
     * @param nLoopKF ?
     * @param bRobust ?
    */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);

    /**
     * @brief 局部 BA
     * 
     * @param pKF 关键帧
     * @param pbStopFlag 停止标志位
     * @param pMap 地图指针
    */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);

    /**
     * @brief 位姿图优化
     * 
     * @param pFrame 帧
    */
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    /**
     * @brief 本质图优化
     * 
     * @param pMap 地图
     * @param pLoopKF 闭环关键帧
     * @param pCurKF 当前关键帧
     * @param NonCorrectedSim3 ？
     * @param CorrectedSim3 ？
     * @param LoopConnections ？
     * @param bFixScale 固定尺度标志位
    */
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    /**
     * @brief 位姿优化？
     * 
     * @param pKF1 关键帧1
     * @param pKF2 关键帧2
     * @param vpMatches1 匹配地图点
     * @param g2oS12 ？
     * @param th2 ？
     * @param bFixScale 固定尺度标志位
    */
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
