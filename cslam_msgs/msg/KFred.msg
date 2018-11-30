uint16 mnId # unique KF id
uint8 mClientId # client id
uint32 mUniqueId
bool mbAck

#Poses
    float32[16] mTcpred
    float32[16] mTcpar

    uint16 mpPred_KfId
    uint8 mpPred_KfClientId

    uint16 mpPar_KfId
    uint8 mpPar_KfClientId

    bool mbServerBA

#Bad flags
    bool mbBad
