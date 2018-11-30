bool bSentOnce

uint16 mnId # unique KF id
uint8 mClientId # client id
uint32 mUniqueId
float64 dTimestamp
bool mbAck

#Grid (to speed up feature matching)
    int16 mnGridCols
    int16 mnGridRows
    float32 mfGridElementWidthInv
    float32 mfGridElementHeightInv

#Calibration parameters
    float32 fx
    float32 fy
    float32 cx
    float32 cy
    float32 invfx
    float32 invfy

#Number of KeyPoints
    int16 N

#KeyPoints, stereo coordinate and descriptors (all associated by an index)
    CvKeyPoint[] mvKeysUn
    Descriptor[] mDescriptors

#Pose relative to parent
    float32[16] mTcpred
    float32[16] mTcpar

    bool mbPoseChanged

    bool mbServerBA

    float32[16] mT_SC

#Scale
    int8 mnScaleLevels
    float32 mfScaleFactor
    float32 mfLogScaleFactor
    float32[8] mvScaleFactors
    float32[8] mvLevelSigma2
    float32[8] mvInvLevelSigma2

#Image bounds and calibration
    int16 mnMinX
    int16 mnMinY
    int16 mnMaxX
    int16 mnMaxY
    float32[9] mK

#MapPoints associated to keypoints
    uint32[] mvpMapPoints_Ids
    uint8[] mvpMapPoints_ClientIds
    uint16[] mvpMapPoints_VectId

#Related KFs
    uint16 mpPred_KfId
    uint8 mpPred_KfClientId
    uint16 mpPar_KfId
    uint8 mpPar_KfClientId

#Bad flags
    bool mbBad
