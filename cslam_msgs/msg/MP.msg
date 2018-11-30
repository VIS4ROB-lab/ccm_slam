bool bSentOnce

uint32 mnId # unique MP id
uint8 mClientId # client id
uint32 mUniqueId
int16 mnFirstKFid
uint8 mnFirstKfClientId
bool mbAck

#Position in absolute coordinates
     float32[3] mPosPred
     float32[3] mPosPar
     bool mbPoseChanged

     bool mbServerBA

#Keyframes observing the point and associated index in keyframe
     uint16[] mObservations_KFIDs
     uint8[] mObservations_KFClientIDs
     uint16[] mObservations_n

#Mean viewing direction
     float32[3] mNormalVector
     bool mbNormalAndDepthChanged

#Best descriptor to fast matching
     uint8[32] mDescriptor

#Reference KeyFrame
     uint16 mpPredKFId
     uint8 mpPredKFClientId
     uint16 mpParKFId
     uint8 mpParKFClientId

#Bad flag
     bool mbBad

#Scale invariance distances
     float32 mfMinDistance
     float32 mfMaxDistance

#Visualization
     bool mbMultiUse
