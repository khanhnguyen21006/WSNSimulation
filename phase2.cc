#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/wsn-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/gnuplot.h"
#include "ns3/point-to-point-module.h"
#include "ns3/callback.h"

#define numSensors 100 // number of sensors in network
#define numCluster 16
#define processTime 1000 // required time for handling fire area completely of each UAV
#define flyingtime 600 // average flying time calculated from phase 1
#define XMAX 4500 // area boundary
#define No 64 // initial uav number from phase 1
#define r 250 // uav's radius

#include <math.h>
#include <vector>
#include <fstream>


using namespace ns3;

NodeContainer c;
double dist;
double coord;

int curUAVs; // current active UAVs

int count,k;
int m,n; // area size

Ptr<Socket> *p2pSocket = new Ptr<Socket>[2];


std::vector<int> timer(No, processTime); // timers of UAVs
std::vector<bool> flag(No, true);

Gnuplot2dDataset nUAVs;



void init(){
  curUAVs = No;
  dist = r*sqrt(2);
  count = 1;  
  m = 8; n = 8;
  k = 0;   
}

bool checkTimer(){
  bool res = true;
	for(int i = 0; i < (int)timer.size(); i++){
    if(timer[i]!=0) {res = false; break;}      
  }
  return res;  	
}

int minTimer(){
  int min = 2000;
  for(int i = 0; i < (int)timer.size(); i++){
    if(timer[i] < min) min = timer[i];      
  }
  return min;   
}

static void send(Ptr<Socket> socket, uint32_t packetSize, uint32_t packetCount, Time packetInterval){	
  if (packetCount > 0){
    
    double t = (double) Simulator::Now().GetSeconds();
    double x = 0.66/0.000876-0.66*exp(-0.000876*(t+flyingtime))/0.000876;
    x = floor(x*10000)/10000; 

    std::ostringstream message;
    message << x;

    Ptr<Packet> packet = Create<Packet>((uint8_t*) message.str().c_str(), packetSize);
    socket->Send (packet);
    std::cout <<"fire coordinator "<<message.str()<<" at "<<Simulator::Now()<<std::endl;
  }
  	
}

void receive(Ptr<Socket> socket){
	Ptr<Packet> packet = socket->Recv();      
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer, packet->GetSize ());
  std::string str = std::string((char*)buffer);

  coord = atof(str.c_str());
  std::cout<<"received "<<coord<<" at "<<Simulator::Now()<<std::endl;  	
}
void add(int temp){
  for(int i = 0; i < temp; i++){
      timer.push_back(processTime);
      flag.push_back(true);
      curUAVs++;      
  }
  std::cout<<"nUAVs: "<<curUAVs<<" at "<<Simulator::Now()<<std::endl;
  nUAVs.Add((double) Simulator::Now().GetSeconds(),curUAVs);
}
void Create2DPlotFile ()
{
  std::string fileNameWithNoExtension = "awrgtrh";
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  std::string plotTitle               = "Number of UAVs over time";
  std::string dataTitle               = "numUAVs";
   
  
  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
  plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Set the labels for each axis.
  plot.SetLegend ("Time (seconds)", "Number of UAVs");

  // Set the range for the x axis.
  plot.AppendExtra ("set xrange [0:3000]");
  plot.AppendExtra ("set yrange [0:125]");

  // Instantiate the dataset, set its title, and make the points be
  // plotted along with connecting lines.
  
  nUAVs.SetTitle (dataTitle);
  nUAVs.SetStyle (Gnuplot2dDataset::LINES_POINTS);

  // Add the dataset to the plot.
  plot.AddDataset (nUAVs);

  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
  
}

static void processData(){

  std::cout<<"Processing......"<<Simulator::Now()<<std::endl;
  for(int i = 0; i < (int)timer.size(); i++){
    if(timer[i]<=200&&flag[i]==true) {
      timer[i]=0;
      flag[i]=false;
      curUAVs--;
    }
    else {
      if(flag[i]==true)
        timer[i]-= 200;          
    }
  }
  std::cout<<"nUAVs: "<<curUAVs<<" at "<<Simulator::Now()<<std::endl;
  nUAVs.Add((double) Simulator::Now().GetSeconds(),curUAVs);
  if(checkTimer()) {
    int temp = 2*(m+n)+8*(count+1)-4;
    if(k!=1){
      add(temp);
      k++;
    }
    else{
      Create2DPlotFile();
      Simulator::Stop ();
    }  
  }
  else {
    int temp = 2*(m+n)+8*count-4;
    if(coord>dist&&k!=1){
      // checkTimer() do sth here;
      
      int min = minTimer();
      if(flyingtime <= min){        
        Simulator::Schedule (Seconds (flyingtime), &add, temp);
 
      } else {
        for(int i = 0; i < (int)temp; i++){
          if(timer[i]==0) {
            timer[i]=processTime; 
            flag[i]==true; 
            curUAVs++;
          }
        }      
      }
      std::cout<<"nUAVs: "<<curUAVs<<" at "<<Simulator::Now()<<std::endl;
      nUAVs.Add((double) Simulator::Now().GetSeconds(),curUAVs);
      count++;
      dist += r*sqrt(2);    
    }
  }  
  
}
 

int main(){
	uint32_t packetSize = 100;
	uint32_t packetNum = 1;
  double interval = 1.0;

  Time interPacketInterval = Seconds (interval); 

	c.Create(2);
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
 	p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

	NetDeviceContainer p2pDevices;
  p2pDevices = p2p.Install (c);

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4Address;
  //NS_LOG_INFO ("Assign IP Addresses.");
  ipv4Address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces ;
  p2pInterfaces = ipv4Address.Assign (p2pDevices);


  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c.Get(0));

  c.Get(0) -> GetObject<ConstantPositionMobilityModel>() -> SetPosition(Vector(0.0, 0.0, 0.0));


  Ptr<ListPositionAllocator> *stationPositionAllocator = new Ptr<ListPositionAllocator>[2];
  stationPositionAllocator[0]= CreateObject<ListPositionAllocator>();
  stationPositionAllocator[0]->Add(Vector(3400.0, 3400.0, 0.0)); // fireX fireY
  mobility.SetPositionAllocator(stationPositionAllocator[0]);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(c.Get(1));
  c.Get(1) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(0.0, 0.0, 0.0)); 
  

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  p2pSocket[0] = Socket::CreateSocket (c.Get(0), tid);
  InetSocketAddress anySourceSocketAddress = InetSocketAddress (Ipv4Address::GetAny (), 80);
  p2pSocket[0]->Bind (anySourceSocketAddress);
  p2pSocket[0]->SetRecvCallback (MakeCallback (&receive));

  p2pSocket[1] = Socket::CreateSocket (c.Get(1), tid); 	
  Ptr<Ipv4> ipv4 = c.Get(0)->GetObject<Ipv4>();
  Ipv4InterfaceAddress uavInterfaceAddress = ipv4->GetAddress(1, 0);
  Ipv4Address uavSinkAddress = uavInterfaceAddress.GetLocal();
  InetSocketAddress uavSocketAddress = InetSocketAddress(uavSinkAddress, 80);
  p2pSocket[1]->Connect(uavSocketAddress);

    //MyModel md;
  Simulator::Schedule (Seconds (0), &init);

  for(int k = 0; k < 15 ; k++){      
      Simulator::Schedule (Seconds (k * 200.0), &send, 
                       p2pSocket[1], packetSize, packetNum, interPacketInterval);
         
      Simulator::Schedule (Seconds (k * 201.0), &processData);
  }
  

  Simulator::Run ();
  Simulator::Destroy ();	

	return 0;

}