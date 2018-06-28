#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>

#include "ns3/gnuplot.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("EnergyExample");

Ptr<Socket> *wifiSocket = new Ptr<Socket>[100];
Ptr<Socket> *p2pSocket = new Ptr<Socket>[1];

Gnuplot2dDataset dataset;
NodeContainer uav, c;

static double getPositionX(Ptr<Node> node){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Vector positon = mobility->GetPosition ();
  return positon.x;

}
static double getPositionY(Ptr<Node> node){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Vector positon = mobility->GetPosition ();
  return positon.y;

}
// trace sink 
static void CourseChange (std::string context, Ptr<const MobilityModel> model){

  Vector position = model->GetPosition ();
  Vector velocity = model->GetVelocity ();
   
  NS_LOG_UNCOND("UAV now in position : x = " << position.x << ", y = " << position.y << " with velocity : x = " << velocity.x << ", y = " << velocity.y<< " at "<<Simulator::Now());
  
}
static void send (Ptr<Socket> socket, uint32_t packetSize, uint32_t packetCount, Time packetInterval){
  
  
  if (packetCount > 0){

    int nodeId = socket->GetNode()->GetId();
    double temp = 200*exp(-0.0000005*(getPositionX(socket->GetNode())-3400)*(getPositionX(socket->GetNode())-3400)-0.0000005*(getPositionY(socket->GetNode())-3400)*(getPositionY(socket->GetNode())-3400))+50;

    std::ostringstream message;

    if(nodeId < 10)
      message << "00" << nodeId;
    else if(nodeId < 100)
      message << "0" << nodeId;
    else 
      message << nodeId;

    if(temp < 100) 
      message << "0" << (int)(temp*1000);
    else 
      message << (int)(temp*1000);

    Ptr<Packet> packet = Create<Packet>((uint8_t*) message.str().c_str(), packetSize);
    socket->Send (packet);
    std::cout <<"Node "<<getPositionX(c.Get(nodeId))<<" "<<getPositionY(c.Get(nodeId))<<" send "<<message.str()<<" at "<<Simulator::Now()<<std::endl;
  }
     
}


static void receive (Ptr<Socket> socket){

  Ptr<Packet> packet = socket->Recv();      
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer, packet->GetSize ());
  std::string str = std::string((char*)buffer);

  int idNode = atoi(str.substr(0,3).c_str());
  

  int sub = atoi(str.substr(3,8).c_str());
  

  // std::ostringstream message;
  // message<<"abcd";
  std::cout<<"Uav received data to sigate "<<idNode<<", data = "<<sub/1000.0<<""" at "<<Simulator::Now()<<std::endl;
 
}


// /// Trace function for remaining energy at UAV.
// void
// RemainingEnergy (double oldValue, double remainingEnergy)
// {
//   NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
//                  << "s Current remaining energy = " << remainingEnergy << "J");
// }

/// Trace function for total energy consumption at UAV.
void
TotalEnergy (double oldValue, double totalEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
  dataset.Add(Simulator::Now ().GetSeconds (), totalEnergy );
}

static void processData(){
  int x, y;
  x = rand() % 4 + 6;
  y = rand() % 4 + 6;
  uav.Get(0) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(x, y, 0.0));
}

void Create2DPlotFile ()
{
  std::string fileNameWithNoExtension = "energy";
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  std::string plotTitle               = "2-D Plot";
  std::string dataTitle               = "Energy over time";

  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
  plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Set the labels for each axis.
  plot.SetLegend ("X Values", "Y Values");

  // Set the range for the x axis.
  plot.AppendExtra ("set xrange [0:+20]");

  // Instantiate the dataset, set its title, and make the points be
  // plotted along with connecting lines.
  
  dataset.SetTitle (dataTitle);
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);


  // Create the 2-D dataset.
  
  // Add the dataset to the plot.
  plot.AddDataset (dataset);

  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
}


int
main (int argc, char *argv[])
{
  /*
  LogComponentEnable ("EnergySource", LOG_LEVEL_DEBUG);
  LogComponentEnable ("BasicEnergySource", LOG_LEVEL_DEBUG);
  LogComponentEnable ("DeviceEnergyModel", LOG_LEVEL_DEBUG);
  LogComponentEnable ("WifiRadioEnergyModel", LOG_LEVEL_DEBUG);
   */

  std::string phyMode ("DsssRate1Mbps");
  
  uint32_t packetSize = 100; // bytes
  uint32_t numPackets = 1;

  double interval = 1.0; // seconds
  bool verbose = false;
  
  double Prss = -80;            // dBm
 
  

  // simulation parameters
  
  //set the transmit power
  
  double offset = 81;

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("Prss", "Intended primary RSS (dBm)", Prss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "Total number of packets to send", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  
  cmd.Parse (argc, argv);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold",
                      StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
                      StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  c.Create (100);     // sensor nodes
  uav.Create (1);

  NodeContainer wifiNodes;
  wifiNodes.Add(uav);
  wifiNodes.Add(c);
  
  
  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (-10));
  wifiPhy.Set ("TxGain", DoubleValue (offset + Prss));
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (0.0));
  

  
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create ();
  wifiPhy.SetChannel (wifiChannelPtr);

  
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue (phyMode), "ControlMode",
                                StringValue (phyMode));
  // Set it to ad-hoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, wifiNodes);

  
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0),
                                 "MinY", DoubleValue (0),
                                 "DeltaX", DoubleValue (500),
                                 "DeltaY", DoubleValue (500),
                                 "GridWidth", UintegerValue (10),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add(Vector(200.0, 200.0, 290.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (uav);
  

  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (5000));
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (uav);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.38));
  //radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.313));
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices.Get(0), sources);
  

  AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper listRouting;
  listRouting.Add (staticRouting, 0);
  listRouting.Add (aodv, 10);


  /** Internet stack **/
  InternetStackHelper internet;
  internet.SetRoutingHelper(listRouting);
  internet.Install (wifiNodes);



  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  p2pSocket[0] = Socket::CreateSocket (uav.Get(0), tid);
  InetSocketAddress anySourceSocketAddress2 = InetSocketAddress (Ipv4Address::GetAny (), 81);
  p2pSocket[0]->Bind (anySourceSocketAddress2);
  p2pSocket[0]->SetRecvCallback (MakeCallback (&receive));

  for(int i = 0; i < 100; i++){
    wifiSocket[i] = Socket::CreateSocket(c.Get(i), tid);
    Ptr<Ipv4> ipv4 = uav.Get(0)->GetObject<Ipv4>();
    Ipv4InterfaceAddress uavInterfaceAddress = ipv4->GetAddress(1, 0);
    Ipv4Address uavSinkAddress = uavInterfaceAddress.GetLocal();
    InetSocketAddress uavSocketAddress = InetSocketAddress(uavSinkAddress, 80);
    wifiSocket[i]->Connect(uavSocketAddress);
  }

  /** connect trace sources **/
  
  std::ostringstream oss;
  oss << "/NodeList/"<< uav.Get (0)->GetId ()<< "/$ns3::MobilityModel/CourseChange";

  Config::Connect (oss.str (), MakeCallback (&CourseChange));

  
  // all sources are connected to UAV
  // // energy source
   Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (0));
  // basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
  // device energy model
  Ptr<DeviceEnergyModel> basicRadioModelPtr =
    basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
  NS_ASSERT (basicRadioModelPtr != NULL);
  basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));
  /***************************************************************************/


  /** simulation setup **/
  for(int k = 0; k < 25 ; k++){
    for(int i = 1; i < 100; i++){
      Simulator::Schedule (Seconds (k * 20.0), &send, 
                       wifiSocket[i], packetSize, numPackets, interPacketInterval);

    }    
    Simulator::Schedule (Seconds (k * 20.0 + 13), &processData);
  }
  


  Simulator::Schedule (Seconds (501.0), &Create2DPlotFile);

  Simulator::Stop (Seconds (502.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
