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


#define numSensors 100
#define numCluster 16
#define numUav 1
#define fireX 3400 // fire coordinates
#define fireY 3400
#define thresholdTemp 100
#define rad 250 // uav's radius

/*
Number of uav = Max {S_tong_bao_phu/S_1_uav_bao_phu, C_tong/C_1_uav (khoi luong tinh toan du lieu tu sensor), L_tong/L_1_uav (transport load: du lieu truyen, data rate, packet...)} | condition: l_uav-uav < L_threshold
*/

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include <list>

using namespace ns3;

NodeContainer sensorNodes, sigateNode;
UAV uav(numCluster, numSensors);
UAV uav1(16, 100);

Ptr<Socket> *wifiSocket = new Ptr<Socket>[numSensors];
Ptr<Socket> *p2pSocket = new Ptr<Socket>[5];

double temperature[numSensors];
std::vector<std::vector<double>> dep;
int n;

Gnuplot3dDataset path, area;

int temp;

//int rxDropCounter, txDropCounter;



NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");

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

double getValue(std::list<double>& l,int n){
  std::list<double>::iterator i = l.begin();
  std::advance(i, n);
  return *i; 
}

/*creating model to store ceasing-event handler*/
class SchedulingModel{
public:
  double rf;
  void Start (void);
  
private:

  void HandleEvent (double eventValue, int t);
};

void
SchedulingModel::Start (void){

  // Ptr<UniformRandomVariable> v = CreateObject<UniformRandomVariable> ();
  // v->SetAttribute ("Min", DoubleValue (10));
  // v->SetAttribute ("Max", DoubleValue (20));
  for(int i = 0; i < n; i++){
    std::vector<double> k = dep[i];
    double t = sqrt(k[0]*k[0] + k[1]*k[1])/10.0;
    rf = t;
    
    Simulator::Schedule (Seconds (t),
                       &SchedulingModel::HandleEvent,
                       this, Simulator::Now ().GetSeconds (), i);
  }
  
}
void
SchedulingModel::HandleEvent (double value, int i){

  std::cout << "Member method received event at "
            << Simulator::Now ().GetSeconds ()
            << "s started at " << value << "s" << std::endl;
  uav1.Get(i) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(0.0, 0.0, 0.0));
}

// trace sink 
static void CourseChange (std::string context, Ptr<const MobilityModel> model){

  Vector position = model->GetPosition ();
  Vector velocity = model->GetVelocity ();
  path.Add(position.x,position.y,290);
   
  NS_LOG_UNCOND("UAV now in position : x = " << position.x << ", y = " << position.y << " with velocity : x = " << velocity.x << ", y = " << velocity.y<< " at "<<Simulator::Now());
  std::ostringstream message;
  if(position.x < 1000)
    message<<"0"<<(int)(position.x*100);
  else
    message<<(int)(position.x*100);
  if(position.y < 1000)
    message<<"0"<<(int)(position.y*100);
  else
    message<<(int)(position.y*100);

  std::cout<<"data = "<<message.str().c_str()<<" at "<<Simulator::Now()<<std::endl;
}

/*counting dropped packet during tranmission*/
// static void MacDrop (Ptr< const Packet > packet){

//   rxDropCounter++;
//   std::cout<<"number of rx packets dropped  = "<<rxDropCounter<<" at "<<Simulator::Now()<<std::endl;
// }
// static void PhyDrop ( Ptr< const Packet > packet){

//   txDropCounter++;
//   std::cout<<"number of tx packets dropped = "<<txDropCounter<<" at "<<Simulator::Now()<<std::endl;
// }

// static void totalDrops (){

//   std::cout<<"Number of UAVs  = "<<n<<std::endl; 
//   std::cout<<"Flying time = "<<532.77<< "(s)"<<std::endl; 
//   std::cout<<"Total delaying time = "<<1181.6127<< "(s)"<<std::endl;
//   std::cout<<"Number of packets dropped: 1 out of 2500"<<std::endl;

// }

//send temp data
static void send (Ptr<Socket> socket, uint32_t packetSize, uint32_t packetCount, Time packetInterval){
  
  uav.x.clear();
  uav.y.clear();
  uav.temp.clear();

  if (packetCount > 0){
    int nodeId = socket->GetNode()->GetId();
    double temp = 200*exp(-0.0000005*(getPositionX(socket->GetNode())-fireX)*(getPositionX(socket->GetNode())-fireX)-0.0000005*(getPositionY(socket->GetNode())-fireY)*(getPositionY(socket->GetNode())-fireY))+50;

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
    std::cout <<"Node "<<getPositionX(sensorNodes.Get(nodeId))<<" "<<getPositionY(sensorNodes.Get(nodeId))<<" send "<<message.str()<<" at "<<Simulator::Now()<<std::endl;
  }
     
}

/*receive data from listening socket*/
static void receive (Ptr<Socket> socket){

  Ptr<Packet> packet = socket->Recv();      
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer, packet->GetSize ());
  std::string str = std::string((char*)buffer);

  int idNode = atoi(str.substr(0,3).c_str());
  uav.x.push_back(getPositionX(sensorNodes.Get(idNode)));
  uav.y.push_back(getPositionY(sensorNodes.Get(idNode)));

  int sub = atoi(str.substr(3,8).c_str());
  uav.temp.push_back(sub/1000.0);

  // std::ostringstream message;
  // message<<"abcd";
  std::cout<<"Uav send back data to sigate "<<idNode<<", data = "<<sub/1000.0<<""" at "<<Simulator::Now()<<std::endl;
  Ptr<Packet> packetBack = Create<Packet>((uint8_t*) str.c_str(), 1000);
  p2pSocket[2]->Send(packetBack);
}

/*forward packet to sigate*/
static void backToSigate (Ptr<Socket> socket){

  Ptr<Packet> packet = socket->Recv();      
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer, packet->GetSize ());
  std::string str = std::string((char*)buffer);

  int idNode = atoi(str.substr(0,3).c_str());
  uav.x.push_back(getPositionX(sensorNodes.Get(idNode)));
  uav.y.push_back(getPositionY(sensorNodes.Get(idNode)));


  int sub = atoi(str.substr(3,8).c_str());
  uav.temp.push_back(sub/1000.0);

  temperature[idNode] = sub/1000.0;

  std::cout<<"Sigate received one packet from UAV node "<<idNode<<" data = "<<sub/1000.0<<""" at "<<Simulator::Now()<<std::endl;
   
}

std::vector<int> convertIndextoPosition(int M, int n, int index){
    std::vector<int> temp;
    //int a = M/n;
    //int mat1[a][n], mat2[a+1][n];
    int x,y;
    // if(M%n == 0){
    //    for(int i = 0; i < a; i++){
    //     for(int j = 0; j < n; j++){
    //         mat1[i][j] = i*n+j;
    //     }
        
    // } 
    // } else {
    //     for(int i = 0; i < a + 1; i++){
    //         for(int j = 0; j < n; j++){
    //             mat2[i][j] = i*n+j;
    //             if(mat2[i][j] > M){
    //                 mat2[i][j] = 0;
    //             }   
    //         }     
    //     }
    // }
    x = index/n;
    y = index%n;
    temp.push_back(x*500);
    temp.push_back(y*500);
    return temp;
}
int max(std::vector<int> vec){
  int max;
  max = vec[0];
  for(unsigned i = 1; i < vec.size(); i++){
    if(vec[i] > max){
      max = vec[i];
    }
  }
  return max;
}
int min(std::vector<int> vec){
  int min;
  min = vec[0];
  for(unsigned i = 1; i < vec.size(); i++){
    if(vec[i] < min)
      min = vec[i];    
  }
  return min;
}
int numberUAV(double width, double height, double range){
    int result;
    result = ceil(width/(range * sqrt(2))) * ceil(height/(range * sqrt(2)));
    return result;
}

/*calculate number of uavs*/
static void request (SchedulingModel *model){

  std::cout<<"Processing... at "<<Simulator::Now()<<std::endl;

  std::vector<int> x, y;
  
  int rectW, rectH;
  for(int i = 0; i < numSensors; i++){
    if(temperature[i] > thresholdTemp){
      std::vector<int> sub = convertIndextoPosition(100, 10, i);
      x.push_back(sub[0]);
      y.push_back(sub[1]);
      
    }
  }
  rectW = max(x) - min(x);
  rectH = max(y) - min(y);
  n = numberUAV(rectW, rectH, rad);
  std::cout<<"Width : "<<rectW<<" maxX : "<<max(x)<<" minX : "<<min(x)<<std::endl;
  std::cout<<"Height : "<<rectH<<" maxY : "<<max(y)<<" minY : "<<min(y)<<std::endl;
  std::cout<<"Number of UAVs : "<<n<<std::endl;

  std::cout<<"End process... at "<<Simulator::Now()<<std::endl;

  double posx,posy,distance, time;
  posx = getPositionX(uav.Get(0));
  posy = getPositionY(uav.Get(0)); 
  distance = sqrt(posx*posx+posy*posy);

  for(int i = 0; i < ceil(rectW/(rad*sqrt(2))); i++){
    for(int j = 0; j < ceil(rectH/(rad*sqrt(2))); j++){
      std::vector<double> v;
      v.push_back(min(x) + rad/sqrt(2) + i*rad*sqrt(2));
      v.push_back(min(y) + rad/sqrt(2) + j*rad*sqrt(2));

      dep.push_back(v);
      area.Add(min(x) + rad/sqrt(2) + i*rad*sqrt(2),min(x) + rad/sqrt(2) + i*rad*sqrt(2),290);
    }
  }
  for(std::vector<double> d : dep){
    for(double k : d){
      std::cout<<k<<" ";     
    }
    std::cout<<"\n";
  }
  for(int i = 0; i < n; i++){
    std::vector<double> k = dep[i];
    
    double x1 = k[0]/sqrt(k[0]*k[0] + k[1]*k[1]);
    double y1 = k[1]/sqrt(k[0]*k[0] + k[1]*k[1]);
    (uav1.Get(i) -> GetObject<ConstantVelocityMobilityModel>()) -> SetVelocity(Vector(x1*10,y1*10,0));
        
  }

  time = distance/10.0;

  std::cout<<"Flying time = "<<time<<" (s)"<<std::endl;

  model->Start();
  

}

void stop(Ptr<Node> cvNode){
  std::cout<<"Stopping Uav\n";
  Ptr<ConstantVelocityMobilityModel> cvMobuav = cvNode->GetObject<ConstantVelocityMobilityModel>();
  cvMobuav-> SetVelocity(Vector(0.0, 0.0, 0.0));  
}

/*matrix manipulating*/
double determinant(double a[6][6],double k)
{
  double s=1,det=0,b[6][6];
  int i,j,m,n,c;
  if (k==1)
    {
     return (a[0][0]);
    }
  else
    {
     det=0;
     for (c=0;c<k;c++)
       {
        m=0;
        n=0;
        for (i=0;i<k;i++)
          {
            for (j=0;j<k;j++)
              {
                b[i][j]=0;
                if (i != 0 && j != c)
                 {
                   b[m][n]=a[i][j];
                   if (n<(k-2))
                    n++;
                   else
                    {
                     n=0;
                     m++;
                     }
                   }
               }
             }
          det=det + s * (a[0][c] * determinant(b,k-1));
          s=-1 * s;
          }
    }
 
    return (det);
}

static void transpose(double num[6][6],double fac[6][6],double inverse[6][6],double r)
{
  int i,j;
  double b[6][6],d;
 
  for (i=0;i<r;i++)
    {
     for (j=0;j<r;j++)
       {
         b[i][j]=fac[j][i];
        }
    }
  d=determinant(num,r);
  for (i=0;i<r;i++)
    {
     for (j=0;j<r;j++)
       {
        inverse[i][j]=b[i][j] / d;
        }
    }
  
}
static void cofactor(double num[6][6],double inverse[6][6],double f)
{
 double b[6][6],fac[6][6];
 int p,q,m,n,i,j;
 for (q=0;q<f;q++)
 {
   for (p=0;p<f;p++)
    {
     m=0;
     n=0;
     for (i=0;i<f;i++)
     {
       for (j=0;j<f;j++)
        {
          if (i != q && j != p)
          {
            b[m][n]=num[i][j];
            if (n<(f-2))
             n++;
            else
             {
               n=0;
               m++;
               }
            }
        }
      }
      fac[q][p]=pow(-1,q + p) * determinant(b,f-1);
    }
  }
  transpose(num,fac,inverse,f);
}

//data processing
static void processData(){

  std::cout<<"Processing......"<<Simulator::Now()<<std::endl;  
  int n = uav.temp.size();
  double X[n][6],X_T[6][n];
  for(int i=0;i<n;i++){
    X_T[0][i]=X[i][0] = 1;
    X_T[1][i]=X[i][1] = getValue(uav.x,i)*getValue(uav.x,i);
    X_T[2][i]=X[i][2] = getValue(uav.y,i)*getValue(uav.y,i);
    X_T[3][i]=X[i][3] = getValue(uav.x,i)*getValue(uav.y,i);
    X_T[4][i]=X[i][4] = getValue(uav.x,i);
    X_T[5][i]=X[i][5] = getValue(uav.y,i);
  }
 double Y[n];
 for(int i=0;i<n;i++) Y[i] = getValue(uav.temp,i);

   double XX_T[6][6];
   double X_TY[6];
 for(int i=0;i<6;i++)
    
   for(int j=0;j<6;j++)
   {
     XX_T[i][j]=0;
     for(int k=0;k<n;k++)
     XX_T[i][j]+=X_T[i][k]*X[k][j];
   }

 for(int i=0;i<6;i++)
   {
     X_TY[i]=0;
     for(int k=0;k<n;k++)
     X_TY[i]+=X_T[i][k]*Y[k];
   }

   double XX_T_inverse [6][6];
   double det = determinant(XX_T,6);
   if(det!=0) cofactor(XX_T,XX_T_inverse,6);
   double L[6];
   for(int i=0;i<6;i++)
   {
     L[i]=0;
     for(int j=0;j<6;j++)
       L[i]+=XX_T_inverse[i][j]*X_TY[j];
   }

   double x = 2*L[1]*getPositionX(uav.Get(0))+L[3]*getPositionY(uav.Get(0))+L[4];
   double y = 2*L[2]*getPositionY(uav.Get(0))+L[3]*getPositionX(uav.Get(0))+L[5];
   double x1=x/sqrt(x*x+y*y);
        double y1=y/sqrt(x*x+y*y);

  (uav.Get(0) -> GetObject<ConstantVelocityMobilityModel>()) -> SetVelocity(Vector(x1*10,y1*10,0));
  
  uav.x.clear();
  uav.y.clear();
  uav.temp.clear();
  
}

void Create3DTemp (){

  std::string fileNameWithNoExtension = "demo";
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  std::string plotTitle               = "3-D Plot";
  std::string dataTitle               = "Temperature data";
  std::string dataTemp                = "Path";
  std::string event                   = "Area boundary";

  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
  plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Rotate the plot 30 degrees around the x axis and then rotate the
  // plot 120 degrees around the new z axis.
  plot.AppendExtra ("set view 50, 70, 1.0, 1.0");

  // Make the zero for the z-axis be in the x-axis and y-axis plane.
  plot.AppendExtra ("set ticslevel 0");

  // Set the labels for each axis.
  plot.AppendExtra ("set xlabel \"Coordination X\"");
  plot.AppendExtra ("set ylabel \"Coordination Y\"");
  plot.AppendExtra ("set zlabel \"Coordination Z\"");

  // Set the ranges for the x and y axis.
  plot.AppendExtra ("set xrange [0:9000]");
  plot.AppendExtra ("set yrange [0:9000]");
  plot.AppendExtra ("set zrange [0:300]");

  // Instantiate the dataset, set its title, and make the points be
  // connected by lines.
  
  Gnuplot3dDataset temp;
  temp.SetTitle (dataTitle);
  path.SetTitle(dataTemp);
  //path1.SetTitle("UAV 2");
  area.SetTitle(event);

  temp.SetStyle("with lines");
  path.SetStyle("with lines");
  area.SetStyle("with lines");
  //path1.SetStyle("with lines");

  // Add the dataset to the plot.
  double x, y, z;
  for(x = 0; x < 10000; x += 100){
    for (y = 0; y < 10000; y += 100)
    {
      z = 200*exp(-0.0000005*(x-fireX)*(x-fireX)-0.0000005*(y-fireX)*(y-fireX))+50;
      temp.Add(x,y,z);

    }
    temp.AddEmptyLine();
  }
  plot.AddDataset(path);
  plot.AddDataset(temp);
  plot.AddDataset(area);

  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
}

int main (int argc, char *argv[]){

  std::string phyMode ("DsssRate1Mbps");
  double distance = 600;  // m
  uint32_t packetSize = 100; // bytes
  uint32_t numPackets = 1;

  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = false;

  CommandLine cmd;


  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("distance", "distance (m)", distance);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
    

  cmd.Parse (argc, argv);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|200|0|200"));

  sensorNodes.Create(numSensors);
  sigateNode.Create(1);
  uav.Create(numUav);
  uav1.Create(100);

  std::cout<<"_______________________________________________________"<<std::endl;


  WifiHelper wifi;
  if (verbose){
    wifi.EnableLogComponents ();  // Turn on all Wifi logging
  }

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NodeContainer p2pNodes;
  p2pNodes.Add(sigateNode);
  p2pNodes.Add(uav);

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (p2pNodes);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
 
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-7) ); 
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  // config wifi channel
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));


  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;  
  wifiMac.SetType ("ns3::AdhocWifiMac");

  NodeContainer wifiNodes;
  wifiNodes.Add(uav);
  wifiNodes.Add(sensorNodes);
  

  NetDeviceContainer wifiDevices;
  wifiDevices = wifi.Install (wifiPhy, wifiMac, wifiNodes);

  AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper listRouting;
  listRouting.Add (staticRouting, 0);
  listRouting.Add (aodv, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (listRouting);   
  internet.Install (wifiNodes);
  internet.Install (sigateNode);

  Ipv4AddressHelper ipv4Address;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4Address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer wifiInterfaces ;
  wifiInterfaces = ipv4Address.Assign (wifiDevices);

  ipv4Address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces ;
  p2pInterfaces = ipv4Address.Assign (p2pDevices); 


  MobilityHelper mobility, mobility1;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0),
                                 "MinY", DoubleValue (0),
                                 "DeltaX", DoubleValue (500),
                                 "DeltaY", DoubleValue (500),
                                 "GridWidth", UintegerValue (10),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (sensorNodes);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (sigateNode);
  sigateNode.Get(0) -> GetObject<ConstantPositionMobilityModel>() -> SetPosition(Vector(0.0, 0.0, 0.0));

  // MobilityHelper *mobUav = new MobilityHelper[numUav];
  // Ptr<ListPositionAllocator> *stationPositionAllocator = new Ptr<ListPositionAllocator>[numUav];
  // for(int i = 0;i < numUav;i++){
  //   stationPositionAllocator[i]= CreateObject<ListPositionAllocator>();  
  // }

  // stationPositionAllocator[0] -> Add(Vector(200, 200, 290));      
  // stationPositionAllocator[1] -> Add(Vector(5000, 2000, 290));       
  // stationPositionAllocator[2] -> Add(Vector(4000, 6000, 290));
  // stationPositionAllocator[3] -> Add(Vector(2000, 7000, 290));     
  
  // for(int i = 0;i < numUav;i++){
  //   mobUav[i].SetPositionAllocator(stationPositionAllocator[i]);
  //   mobUav[i].SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  //   mobUav[i].Install(uav.Get(i));
  //   uav.Get(i) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(10.0, 10.0, 0.0)); 
  // }

  Ptr<ListPositionAllocator> *stationPositionAllocator = new Ptr<ListPositionAllocator>[2];
  stationPositionAllocator[0]= CreateObject<ListPositionAllocator>();
  stationPositionAllocator[0]->Add(Vector(200, 200, 290));
  stationPositionAllocator[1]= CreateObject<ListPositionAllocator>();
  stationPositionAllocator[1]->Add(Vector(0, 0, 290));

  mobility.SetPositionAllocator(stationPositionAllocator[0]);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(uav.Get(0));
  uav.Get(0) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(10.0, 10.0, 0.0)); 

  for(int i = 0; i < 100; i++){
    mobility1.SetPositionAllocator(stationPositionAllocator[1]);
    mobility1.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility1.Install(uav1.Get(i));
  }

  // Ptr<ListPositionAllocator> stationPositionAllocator = CreateObject<ListPositionAllocator>();
  // stationPositionAllocator->Add(Vector(200, 200, 290));


  // mobility.SetPositionAllocator(stationPositionAllocator);
  // mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  // mobility.Install(uav.Get(0));
  // uav.Get(0) -> GetObject<ConstantVelocityMobilityModel>() -> SetVelocity(Vector(10.0, 10.0, 0.0)); 

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  p2pSocket[0] = Socket::CreateSocket (uav.Get(0), tid);
  InetSocketAddress anySourceSocketAddress = InetSocketAddress (Ipv4Address::GetAny (), 80);
  p2pSocket[0]->Bind (anySourceSocketAddress);
  p2pSocket[0]->SetRecvCallback (MakeCallback (&receive));

  p2pSocket[1] = Socket::CreateSocket (sigateNode.Get(0), tid);
  InetSocketAddress anySourceSocketAddress2 = InetSocketAddress (Ipv4Address::GetAny (), 81);
  p2pSocket[1]->Bind (anySourceSocketAddress2);
  p2pSocket[1]->SetRecvCallback (MakeCallback (&backToSigate));

  for(int i = 0; i < numSensors; i++){
    wifiSocket[i] = Socket::CreateSocket(sensorNodes.Get(i), tid);
    Ptr<Ipv4> ipv4 = uav.Get(0)->GetObject<Ipv4>();
    Ipv4InterfaceAddress uavInterfaceAddress = ipv4->GetAddress(1, 0);
    Ipv4Address uavSinkAddress = uavInterfaceAddress.GetLocal();
    InetSocketAddress uavSocketAddress = InetSocketAddress(uavSinkAddress, 80);
    wifiSocket[i]->Connect(uavSocketAddress);
  }

  p2pSocket[2] =  Socket::CreateSocket(uav.Get(0), tid);
  Ptr<Ipv4> ipv4Sigate = sigateNode.Get(0)->GetObject<Ipv4>();
  Ipv4InterfaceAddress sigateInterfaceAddress = ipv4Sigate->GetAddress(1, 0);
  Ipv4Address sigateSinkAddress = sigateInterfaceAddress.GetLocal();
  InetSocketAddress sigateSocketAddress = InetSocketAddress(sigateSinkAddress, 81);
  p2pSocket[2]->Connect(sigateSocketAddress);
  
  if (tracing == true){
    wifiPhy.EnablePcapAll ("demo");
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
    aodv.PrintRoutingTableAllEvery (Seconds (2), routingStream);
    Ptr<OutputStreamWrapper> neighborStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.neighbors", std::ios::out);
    aodv.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);
    
  }

  SchedulingModel model;

  std::ostringstream oss;
  oss << "/NodeList/"<< uav.Get (0)->GetId ()<< "/$ns3::MobilityModel/CourseChange";
  Config::Connect (oss.str (), MakeCallback (&CourseChange));

  for(int k = 0; k < 100 ; k++){
    std::ostringstream oss;
    oss << "/NodeList/"<< uav1.Get (k)->GetId ()<< "/$ns3::MobilityModel/CourseChange";
    Config::Connect (oss.str (), MakeCallback (&CourseChange));
  }

  /*Node config for counting packets*/
  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacDrop));
  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop ", MakeCallback(&PhyDrop));



  for(int k = 0; k < 25 ; k++){
    for(int i = 1; i < numSensors; i++){
      Simulator::Schedule (Seconds (k * 20.0), &send, 
                       wifiSocket[i], packetSize, numPackets, interPacketInterval);

    }    
    Simulator::Schedule (Seconds (k * 20.0 + 13), &processData);
  }

  Simulator::Schedule (Seconds (500.0), &stop, uav.Get(0));
  
  Simulator::Schedule (Seconds (501.0), &request, &model);

  Simulator::Schedule (Seconds (1300.0), &Create3DTemp);
  //Simulator::Schedule (Seconds (1300.0), &totalDrops);


  std::cout<<"Starting simulation ........."<<std::endl;
  AnimationInterface anim("mophong.xml");
  anim.UpdateNodeColor (uav.Get (0), 0, 255, 0);
  
  //anim.SetMaxPktsPerTraceFile(50000);
  Simulator::Stop (Seconds (1301.0));
  Simulator::Run ();
  Simulator::Destroy ();


  return 0;
    
}

