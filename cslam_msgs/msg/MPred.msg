uint32 mnId # unique MP id
uint8 mClientId # client id
uint32 mUniqueId
bool mbAck

#Position in absolute coordinates
     float32[3] mPosPred
     float32[3] mPosPar

     bool mbNormalAndDepthChanged

    bool mbServerBA

#Reference KeyFrame
     uint16 mpPredKFId
     uint8 mpPredKFClientId
     uint16 mpParKFId
     uint8 mpParKFClientId

#Bad flag
     bool mbBad

#Visualization
     bool mbMultiUse
