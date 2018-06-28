/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef WSN_H
#define WSN_H
#include "ns3/node-container.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/packet.h"
#include "ns3/node-container.h"


#include <iostream>
#include <list>
namespace ns3 {
class Sensor:public NodeContainer {

public:
  	
	Sensor();
	~Sensor();
	int num;
	//NodeContainer nodes;
	Ptr<Node> Cluster;
	double *temp;
	double *hump;
	void setCluster(Ptr<Node>Cluster);
	Ptr<Node> getCluster(void);
	//void send(uint32_t pktSize,uint32_t pktCount);
	//Ptr<Socket> *source;
	//Ptr<Socket> *sink;
	std::ostringstream data;
	
};

class Cluster: public NodeContainer {

public:
  	
	Cluster(int numCluster,int numNode);
	~Cluster();
std::ostringstream* data;	// void setNode(Ptr<Node> n);
	//NodeContainer nodes;
	// Ptr<Node> Cluster;
	// double **temp;
	// double **hump;
	//void setCluster(Ptr<Node>Cluster,Ptr<Node> *Node);
	// std:ostringstream data;
	//void send(uint32_t pktSize,uint32_t pktCount);
	//Ptr<Socket> *source;
	//Ptr<Socket> *sink;
	
	
};
/* ... */
class UAV: public NodeContainer{
	
	public: 
		UAV(int numCluster,int numNode);
		~UAV();
		std::list<double> temp;
	 std::list<double> x;
	 std::list<double> y;
		int numClus;
	int numNodes;
	std::string text;
};



class Temp: public Packet{
public:
	Temp(std::map<int,double> m);
	std::map<int,double> map;
	std::map<int,double> getData(void);

};

}

#endif /* WSN_H */

