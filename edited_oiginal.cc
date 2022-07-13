//
// @date: 08-11-2015
// @author: Asanga Udugama (adu@comnets.uni-bremen.de)
//

#include "KWirelessInterface1.h"

#include "KBaseNodeInfo.h"


Define_Module(KWirelessInterface1);

void KWirelessInterface1::initialize(int stage)
{
    if (stage == 0) {

        // get parameters
        ownMACAddress = par("ownMACAddress").stringValue();
        wirelessRange = par("wirelessRange");
        neighbourScanInterval = par("neighbourScanInterval");
        bandwidthBitRate = par("bandwidthBitRate");
        wirelessHeaderSize = par("wirelessHeaderSize");
        advertisementWindow = par("advertisementWindow");
        numChannel = par("numChannel");
        advertisementInterval = advertisementWindow*numChannel;
        neighbourScanWindow = neighbourScanInterval/numChannel;
        usedRNG = par("usedRNG");
        timingDelay = par("timingDelay");
        Ts = par("Ts");
        dutyCycle = neighbourScanInterval/neighbourScanWindow;
        advIND = 0.000128;
        //startsimtime = par("startsimtime");
        //endsimtime = par("endsimtime");
        // set other parameters
        broadcastMACAddress = "FF:FF:FF:FF:FF:FF";

    } else if (stage == 1) {
        // cal sim time
        //getClockTime(CLOCK_MONOTONIC, &startsimtime);
        // get own module info
        ownNodeInfo = new KBaseNodeInfo();
        ownNodeInfo->nodeModule = getParentModule();
        cModule *unknownModule = getParentModule()->getSubmodule("mobility");
        ownNodeInfo->nodeMobilityModule = check_and_cast<inet::IMobility*>(unknownModule);
        ownNodeInfo->nodeWirelessIfcModule = this;
        ownNodeInfo->scnWindow = this->neighbourScanInterval; //since we have one channel and there is no sleep time

    } else if (stage == 2) {

        // get module info of all other nodes in network
        for (int id = 0; id <= getSimulation()->getLastComponentId(); id++) {
            cModule *unknownModule = getSimulation()->getModule(id);
            if (unknownModule == NULL) {
                continue;
            }

            // if module is a KNode or KHeraldNode but is yourself
            if (unknownModule == ownNodeInfo->nodeModule) {
                continue;
            }

            // find the wireless ifc module & mobility module
            cModule *unknownMobilityModule = NULL;
            cModule *unknownWirelessIfcModule = NULL;

            for (cModule::SubmoduleIterator it(unknownModule); !it.end(); ++it) {

                if (dynamic_cast<inet::IMobility*>(*it) != NULL) {
                    unknownMobilityModule = *it;
                }
                if (dynamic_cast<KWirelessInterface1*>(*it) != NULL) {
                    unknownWirelessIfcModule = *it;
                }
            }

            if (unknownMobilityModule != NULL && unknownWirelessIfcModule != NULL) {
                KBaseNodeInfo *nodeInfo = new KBaseNodeInfo();
                nodeInfo->nodeModule = unknownModule;
                nodeInfo->nodeMobilityModule = dynamic_cast<inet::IMobility*>(unknownMobilityModule);
                nodeInfo->nodeWirelessIfcModule = dynamic_cast<KWirelessInterface1*>(unknownWirelessIfcModule);
                double otherNodeTime = nodeInfo->nodeWirelessIfcModule->par("neighbourScanInterval").doubleValue(); // Record the Scan interval of other nodes
                allNodeInfoList.push_back(nodeInfo);
            }
        }

        // setup first event to build neighbourhood node list and send to forwarding layer
        cMessage *sendNeighEvent = new cMessage("Send Neighbourhood Event");
        sendNeighEvent->setKind(KWIRELESSINTERFACE1_NEIGH_EVENT_CODE);
        scheduleAt(simTime() + neighbourScanInterval, sendNeighEvent);


        // setup pkt send event message
        sendPacketTimeoutEvent = new cMessage("Send Packet Timeout Event");
        sendPacketTimeoutEvent->setKind(KWIRELESSINTERFACE1_PKTSEND_EVENT_CODE);

        // setup statistics signals
        neighSizeSignal = registerSignal("linkNeighSize");
        neighSizeCountSignal = registerSignal("linkNeighSizeCount");
        contactDurationSignal = registerSignal("linkContactDuration");
        contactDurationCountSignal = registerSignal("linkContactDurationCount");
        //Model Statistics
        discoveryDelay1 = registerSignal("delay");
        //discoveryDelay2 = registerSignal("delay2");
        //probabilitySucc =registerSignal("linkPacketProbabilityCount");
        probabilityUnSucc = registerSignal("linkNonSuccIdProb");
        probabilityofSuccessCountSignal = registerSignal("linkPacketProbabilityCount");
        discoveryProbability = registerSignal("linkdiscoveryProbability");



    } else {
        EV_FATAL <<  KWIRELESSINTERFACE1_SIMMODULEINFO << "Something is radically wrong\n";
    }
}

int KWirelessInterface1::numInitStages() const
{
    return 3;
}

void KWirelessInterface1::handleMessage(cMessage *msg)
{

    // find and send neighbour list to upper layer
    if (msg->isSelfMessage() && msg->getKind() == KWIRELESSINTERFACE1_NEIGH_EVENT_CODE) {

        // init current neighbor list
        while (currentNeighbourNodeInfoList.size() > 0) {
            list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
            KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo;
            currentNeighbourNodeInfoList.remove(nodeInfo);
        }

        // get current position of self
        inet::Coord ownCoord = ownNodeInfo->nodeMobilityModule->getCurrentPosition();

        // check which nodes are neighbours and if so, add to list
        list<KBaseNodeInfo*>::iterator iteratorAllNodeInfo = allNodeInfoList.begin();
        while (iteratorAllNodeInfo != allNodeInfoList.end()) {
            KBaseNodeInfo *nodeInfo = *iteratorAllNodeInfo;

            neighbourScanIntervalTrigger = simTime()+neighbourScanInterval;
            neighbourScanIntervalTrigger_DBL = SIMTIME_DBL(neighbourScanIntervalTrigger);
            nodeInfo->scnWindow = neighbourScanIntervalTrigger_DBL; // Update the scanWindow trigger time of the node, so we can also share it with other nodes

            inet::Coord neighCoord = nodeInfo->nodeMobilityModule->getCurrentPosition();

#ifdef KWIRELESSINTERFACE1_EUCLIDEAN_DISTANCE
            // check using euclidean distances
            double l = ((neighCoord.x - ownCoord.x) * (neighCoord.x - ownCoord.x))
                + ((neighCoord.y - ownCoord.y) * (neighCoord.y - ownCoord.y));
            double r = wirelessRange * wirelessRange;

            if (l <= r) {
                currentNeighbourNodeInfoList.push_back(nodeInfo);
            }
#else
            // check using chebyshev and euclidean distances
            double l = MAX(POSITIVE(neighCoord.x - ownCoord.x), POSITIVE(neighCoord.y - ownCoord.y));
            if (l <= wirelessRange) {
                l = ((neighCoord.x - ownCoord.x) * (neighCoord.x - ownCoord.x))
                    + ((neighCoord.y - ownCoord.y) * (neighCoord.y - ownCoord.y));
                double r = wirelessRange * wirelessRange;
                if (l <= r) {
                    currentNeighbourNodeInfoList.push_back(nodeInfo);
                }
            }
#endif
            iteratorAllNodeInfo++;
        }

        // compute and emit stats
#ifdef KWIRELESSINTERFACE1_COMPUTE_STATS
        generateStats();
#endif
        // if there are neighbours, send message
        if (currentNeighbourNodeInfoList.size() > 0) {

            // build message
            int neighCount = 0;

            KNeighbourListMsg *neighListMsg = new KNeighbourListMsg("Neighbour List Msg");
            neighListMsg->setNeighbourNameListArraySize(currentNeighbourNodeInfoList.size());
            neighListMsg->setNeighbourNameCount(currentNeighbourNodeInfoList.size());

            list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
            while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList.end()) {
                KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo;

                string nodeAddress = nodeInfo->nodeModule->par("ownAddress").stringValue();
                neighListMsg->setNeighbourNameList(neighCount, nodeAddress.c_str());

                neighCount++;
                iteratorCurrentNeighbourNodeInfo++;
            }

            // send msg to upper layer
            send(neighListMsg, "upperLayerOut");

        }


        // setup next event to build neighbourhood node list and send to forwarding layer
        cMessage *sendNeighEvent = new cMessage("Send Neighbourhood Event");
        sendNeighEvent->setKind(KWIRELESSINTERFACE1_NEIGH_EVENT_CODE);
        scheduleAt(simTime() + neighbourScanInterval, sendNeighEvent);


        delete msg;

    // trigger to send pending packet and setup new send
    } else if (msg->isSelfMessage() && msg->getKind() == KWIRELESSINTERFACE1_PKTSEND_EVENT_CODE) {

        // send the pending packet out
        sendPendingMsg();

        // if there are queued packets, setup for sending the next one at top of queue
        if (!packetQueue.empty()) {

            // get next at the top of queue
            cMessage *nextMsg = packetQueue.front();
            packetQueue.pop();

            // setup for next message sending and start timer
            setupSendingMsg(nextMsg);
        }
    }


    // process a packet (arriving from upper or lower layers)
    else {

        cGate *gate;
        char gateName[32];

       // get message arrival gate name
        gate = msg->getArrivalGate();
        strcpy(gateName, gate->getName());

        // msg from upper layer
        if (strstr(gateName, "upperLayerIn") != NULL) {


            // if currently there is a pending msg, then queue this msg
            if (sendPacketTimeoutEvent->isScheduled()) {

                packetQueue.push(msg);

            // no queued msgs
            } else {

                // so setup for next message sending and start timer
                setupSendingMsg(msg);

              }

        // from lowerLayerIn
        } else {

            send(msg, "upperLayerOut");

        }
    }
}

void KWirelessInterface1::setupSendingMsg(cMessage *msg)
{
    string destinationAddress = getDestinationAddress(msg);
    bool isBroadcastMsg = FALSE;
    if (destinationAddress == broadcastMACAddress) {
        isBroadcastMsg = TRUE;
    }

    // make the neighbour list at begining of msg tx (to check later if those neighbours are still there)
    list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
    while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList.end()) {
        KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo;
        string nodeAddress = nodeInfo->nodeModule->par("ownAddress").stringValue();

        // if broadcast, add all addresses to tx time neighbour list
        // if unicast, add only the specific address
        if (isBroadcastMsg || destinationAddress == nodeAddress) {
            atTxNeighbourNodeInfoList.push_back(nodeInfo);
        }

        iteratorCurrentNeighbourNodeInfo++;
    }

    // save the msg to send
    currentPendingMsg = msg;

    // compute transmission duration
    cPacket *currentPendingPkt = dynamic_cast<cPacket*>(currentPendingMsg);
    double bitsToSend = (currentPendingPkt->getByteLength() * 8) + (wirelessHeaderSize * 8);
    double txDuration = bitsToSend / bandwidthBitRate;

    // setup timer to trigger at tx duration
    //padding = advertisementWindow;// i want the advert and Scan to trigger at same time using this padding
    randomDelay  = uniform(0,timingDelay,usedRNG);
    advertismentPeriod  = randomDelay + advertisementInterval;
    scheduleAt(simTime() + advertisementWindow + randomDelay, sendPacketTimeoutEvent);
    //advertismentPeriodTrigger = sendPacketTimeoutEvent->getArrivalTime();
    advertismentPeriodTrigger = simTime() + advertisementWindow + randomDelay;
    advertismentPeriodTrigger_DBL = SIMTIME_DBL(advertismentPeriodTrigger);// Update the time for a new advertisement period


}

void KWirelessInterface1::sendPendingMsg()
{
    // check if nodes to deliver are still in neighbourhood, if so send the packet
    list<KBaseNodeInfo*>::iterator iteratorAtTxNeighbourNodeInfo = atTxNeighbourNodeInfoList.begin();
    while (iteratorAtTxNeighbourNodeInfo != atTxNeighbourNodeInfoList.end()) {
        KBaseNodeInfo *atTxNeighbourNodeInfo = *iteratorAtTxNeighbourNodeInfo;
        string atTxNeighbourNodeAddress = atTxNeighbourNodeInfo->nodeModule->par("ownAddress").stringValue();

        list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
        while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList.end()) {
            KBaseNodeInfo *currentNeighbourNodeInfo = *iteratorCurrentNeighbourNodeInfo;
            string currentNeighbourNodeAddress = currentNeighbourNodeInfo->nodeModule->par("ownAddress").stringValue();

            // check if node is still in neighbourhood
            if (atTxNeighbourNodeAddress == currentNeighbourNodeAddress) {



                // Check probability of successful transmission

                postertioriScantime = currentNeighbourNodeInfo->scnWindow - neighbourScanWindow ;
                postertioriAdvInterval = advertismentPeriodTrigger_DBL - advertisementInterval - randomDelay ;
                residualTime = advertismentPeriodTrigger_DBL- randomDelay -  advIND - postertioriScantime;
                residualTime2 = currentNeighbourNodeInfo->scnWindow - (advertismentPeriodTrigger_DBL + advIND);
                cout <<residualTime<< " "<<residualTime2<<" " <<Ts<<endl;
                if( (residualTime >= Ts or residualTime2 >= Ts)) {
                    cout<<"Successful"<<endl;
                    probability = (1/numChannel)*((K-1)*(advertisementWindow/neighbourScanInterval)+dutyCycle-(K-1)*(advertisementWindow/neighbourScanInterval)-(Ts/neighbourScanInterval));
                    cout<<probability<<endl;

                    //cout<<currentNeighbourNodeInfo->nodeModule->getId()<<",";
                    //cout<<"node_"<<currentNeighbourNodeInfo->nodeModule->getId()<<" successfully pairs "<<myCounter<<" time(s)" <<endl;

                }
                else {
                    cout<<"Unsuccessful"<<endl;
                    //probUnsuccessful= 1;
                }


                //emit(probabilityofSuccessCountSignal,1);
                emit(discoveryProbability, probability);

                //Send message if the probability is 1
                if(residualTime >= Ts or residualTime2 >= Ts){

                    model1 = advertisementInterval + randomDelay + advIND;
                    //cout<<advIND<<endl;
                    //cout<<model1<<endl;


                    emit(discoveryDelay1,model1);
                    //emit(discoveryDelay2,model1);
                    //emit(totalBeaconDelayTime2,model2);

                    // make duplicate of packet
                    cPacket *outPktCopy =  dynamic_cast<cPacket*>(currentPendingMsg->dup());


                    // send to node
                    sendDirect(outPktCopy, currentNeighbourNodeInfo->nodeModule, "radioIn");
                }

                break;
            }

            iteratorCurrentNeighbourNodeInfo++;

        }

        iteratorAtTxNeighbourNodeInfo++;

    }

    // remove original message
    delete currentPendingMsg;
    currentPendingMsg = NULL;

    // remove entries in list used to check neighbour list at begining of msg tx
    while (atTxNeighbourNodeInfoList.size() > 0) {
        list<KBaseNodeInfo*>::iterator iteratorAtTxNeighbourNodeInfo = atTxNeighbourNodeInfoList.begin();
        KBaseNodeInfo *nodeInfo = *iteratorAtTxNeighbourNodeInfo;
        atTxNeighbourNodeInfoList.remove(nodeInfo);
    }

}

string KWirelessInterface1::getDestinationAddress(cMessage *msg)
{
    KDataMsg *dataMsg = dynamic_cast<KDataMsg*>(msg);
    if (dataMsg) {
        return dataMsg->getDestinationAddress();
    }

    KFeedbackMsg *feedbackMsg = dynamic_cast<KFeedbackMsg*>(msg);
    if (feedbackMsg) {
        return feedbackMsg->getDestinationAddress();
    }

    KSummaryVectorMsg *summaryVectorMsg = dynamic_cast<KSummaryVectorMsg*>(msg);
    if (summaryVectorMsg) {
        return summaryVectorMsg->getDestinationAddress();
    }

    KDataRequestMsg *dataRequestMsg = dynamic_cast<KDataRequestMsg*>(msg);
    if (dataRequestMsg) {
        return dataRequestMsg->getDestinationAddress();
    }

    KDPtableRequestMsg *dpTableRequestMsg = dynamic_cast<KDPtableRequestMsg*>(msg);
    if (dpTableRequestMsg) {
        return dpTableRequestMsg->getDestinationAddress();
    }

    KDPtableDataMsg *dpTableDataMsg = dynamic_cast<KDPtableDataMsg*>(msg);
    if (dpTableDataMsg) {
        return dpTableDataMsg->getDestinationAddress();
    }


    EV_FATAL <<  KWIRELESSINTERFACE1_SIMMODULEINFO << ">!<Unknown message type. Check \"string KWirelessInterface1::getDestinationAddress(cMessage *msg)\"\n";

    throw cRuntimeError("Unknown message type in KWirelessnterface");

    return string();
}


void KWirelessInterface1::generateStats()
{

    emit(neighSizeSignal, static_cast<long>(currentNeighbourNodeInfoList.size()));
    emit(neighSizeCountSignal, 1);

    // check and remove left neighbourhood
    list<KBaseNodeInfo*>::iterator itPrevNeighNodeInfo = previousNeighbourNodeInfoList.begin();
    while (itPrevNeighNodeInfo != previousNeighbourNodeInfoList.end()) {
        KBaseNodeInfo *prevNodeInfo = *itPrevNeighNodeInfo;

        list<KBaseNodeInfo*>::iterator itCurrNeighNodeInfo = currentNeighbourNodeInfoList.begin();
        bool found = FALSE;
        while (itCurrNeighNodeInfo != currentNeighbourNodeInfoList.end()) {
            KBaseNodeInfo *currNodeInfo = *itCurrNeighNodeInfo;

            if (currNodeInfo->nodeAddress == prevNodeInfo->nodeAddress) {
                found = TRUE;
                break;
            }

            itCurrNeighNodeInfo++;
        }
        if (!found) {
            simtime_t duration = simTime() - prevNodeInfo->neighbourStartTime;
            emit(contactDurationSignal, duration);
            emit(contactDurationCountSignal, 1);
            prevNodeInfo->neighbourStartTime = -1.0;
        }

        itPrevNeighNodeInfo++;
    }
    bool found = TRUE;
    while (previousNeighbourNodeInfoList.size() > 0 && found) {
        found = FALSE;
        list<KBaseNodeInfo*>::iterator itPrevNeighNodeInfo = previousNeighbourNodeInfoList.begin();
        while (itPrevNeighNodeInfo != previousNeighbourNodeInfoList.end()) {
            KBaseNodeInfo *prevNodeInfo = *itPrevNeighNodeInfo;
            if(prevNodeInfo->neighbourStartTime == -1.0) {
                previousNeighbourNodeInfoList.remove(prevNodeInfo);
                found = TRUE;
                break;
            }
            itPrevNeighNodeInfo++;
        }
    }

    // insert new neighbours
    list<KBaseNodeInfo*>::iterator itCurrNeighNodeInfo = currentNeighbourNodeInfoList.begin();
    while (itCurrNeighNodeInfo != currentNeighbourNodeInfoList.end()) {
        KBaseNodeInfo *currNodeInfo = *itCurrNeighNodeInfo;

        list<KBaseNodeInfo*>::iterator itPrevNeighNodeInfo = previousNeighbourNodeInfoList.begin();
        bool found = FALSE;
        while (itPrevNeighNodeInfo != previousNeighbourNodeInfoList.end()) {
            KBaseNodeInfo *prevNodeInfo = *itPrevNeighNodeInfo;

            if (prevNodeInfo->nodeAddress == currNodeInfo->nodeAddress) {
                found = TRUE;
                break;
            }

            itPrevNeighNodeInfo++;
        }
        if (!found) {
            currNodeInfo->neighbourStartTime = simTime();
            previousNeighbourNodeInfoList.push_back(currNodeInfo);
        }

        itCurrNeighNodeInfo++;
    }
}

void KWirelessInterface1::finish()
{
    // remove send msg timeout
    if (sendPacketTimeoutEvent->isScheduled()) {
        cancelEvent(sendPacketTimeoutEvent);
    }
    delete sendPacketTimeoutEvent;


    // remove all messages
    while(!packetQueue.empty()) {
        cMessage *nextMsg = packetQueue.front();
        packetQueue.pop();
        delete nextMsg;
    }

    if (currentPendingMsg != NULL) {
        delete currentPendingMsg;
        currentPendingMsg = NULL;
    }
    // sim end time
  //  clock_gettime(CLOCK_MONOTONIC, &endsimtime);
   // double time_taken;
   // time_taken = (endsimtime.tv_sec - startsimtime.tv_sec) * 1e9;
    //time_taken = (time_taken + (endsimtime.tv_nsec - startsimtime.tv_nsec)) * 1e-9;

    //cout << "Time taken by program is : " << fixed
      //   << time_taken << setprecision(9);
   // cout << " sec" << endl;
}

