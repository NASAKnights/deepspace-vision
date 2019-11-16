class timeGraph  {
public:
  //--------------------------------------------------------------------------------
  timeGraph(string name,  int size ) {
    GraphSize=size;
    graph = new TGraph(GraphSize);
    GraphX.resize(GraphSize,0); GraphY.resize(GraphSize,0);
    sprintf(Gname,"%s",name.c_str());
    graph->SetName(Gname);
    t0=time(NULL);
    for (int ti=0; ti<GraphSize; ti++) {
      GraphX[ti]=t0-600+ti;
      GraphY[ti]=0;
      graph->SetPoint(ti,GraphX[ti],GraphY[ti]);
    }
    graph->SetMarkerStyle(20);
    graph->SetMarkerColor(kBlue);
    graph->SetMarkerSize(0.5);

    graph->GetXaxis()->SetTitle("time ");
    graph->GetYaxis()->SetTitle("a.u.");
    //graph->SetMinimum(0);
    graph->SetTitle(Gname);
    graph->GetXaxis()->SetTimeDisplay(1);
    graph->GetXaxis()->SetLabelSize(0.03);
    time_t start_time = time(NULL);
    //start_time = 3600* (int)(start_time/3600);
    gStyle->SetTimeOffset(start_time);
    //printf("timeGraph:: set time offset: %lu,  time now=%lu\n",start_time,time(0));
    //graph->GetXaxis()->SetTimeFormat("y. %Y %F2000-01-01 00:00:00");

    //return graph;
  }
  //--------------------------------------------------------------------------------
  int fillGraph(double X , double Y ) {
    for (Int_t i=0;i<GraphSize-1;i++) {
      GraphX[i] = GraphX[i+1];
      GraphY[i] = GraphY[i+1];
    }
    GraphX[GraphSize-1] =  Gtime++; //X;
    GraphY[GraphSize-1] = Y;
    return 0;
  }
  //--------------------------------------------------------------------------------
  int fillGraph2(double X , double Y ) {
    //printf("fillGraph2: x=%f y=%f \n",X,Y);
    for (Int_t i=0;i<GraphSize-1;i++) {
      GraphX[i] = GraphX[i+1];
      GraphY[i] = GraphY[i+1];
      graph->SetPoint(i,GraphX[i],GraphY[i]);
    }
    GraphX[GraphSize-1] = X;//-t0; // Gtime++;   // X;
    GraphY[GraphSize-1] = Y;
    graph->SetPoint(GraphSize-1,GraphX[GraphSize-1],GraphY[GraphSize-1]);
    //printf("tv: sec=%lu  usec=%lu\n",Time.tv_sec,Time.tv_usec);
    return 0;
  }

public:
  //----  Graph ----
  TGraph   *graph;
  int GraphSize;
  char Gname[256];
  vector<double> GraphX;
  vector<double> GraphY;
  int GraphSrc;
  int GraphXbin;
  int GraphYbin;
  unsigned int Gtime;
  time_t t0;

};


int t_graph() {

  TSocket *sock;
  while (true) {
    //sock = new TSocket("192.168.1.242", 6969);
    sock = new TSocket("10.1.22.91", 6969);
    //sock = new TSocket(host, 6969);
    if (sock->IsValid()) break;
    printf("wait for socket  ... \n");
    sleep(1);
  }

  //sock->Recv(str, 32);
  //  sock->SetCompressionLevel(1);
  //     sock->Send(mess);          // send message
  //sock->Send("Finished");          // tell server we are finished
  // Close the socket
  //sock->Close();
  printf("Connected\n");
  char line[120];
  double rangemin[10];
  double rangemax[10];
  
  rangemin[0] = -30;
  rangemax[0] = 30;

  rangemin[1] = 0;
  rangemax[1] = 250;

  rangemin[2] = 0;
  rangemax[2] = 100;

  rangemin[3] = -45;
  rangemax[3] = 45;

  rangemin[4] = 0;
  rangemax[4] = 100;

  rangemin[5] = -200;
  rangemax[5] = 200;

  rangemin[6] = -200;
  rangemax[6] = 200;

  rangemin[7] = 0;
  rangemax[7] = 0;

  rangemin[8] = 0;
  rangemax[8] = 0;

  rangemin[9] = 0;
  rangemax[9] = 0;

  
  timeGraph*  gr[10];
  int Glen=600;
  gr[0] = new timeGraph("X",Glen );
  gr[1] = new timeGraph("Y",Glen );
  gr[2] = new timeGraph("Z",Glen );
  gr[3] = new timeGraph("#alpha",Glen );
  gr[4] = new timeGraph("Dist",Glen );
  gr[5] = new timeGraph("Offset X",Glen );
  gr[6] = new timeGraph("Offset Y",Glen );
  gr[7] = new timeGraph("X-Z position",Glen );

  int Ngraph=7;

  for (int k=0; k<Ngraph; k++) {
    //gr[k]->graph->Draw("alp");
  }

  TCanvas* c1 = new TCanvas("c1","Robot Telemetry",50,10,1500,1000);
  c1->Divide(2,Ngraph/2+1);
  TCanvas* c2 = new TCanvas("c2","Robot Tracking",20,50,500,500);

  while (true) {
      
    TTimeStamp now;
    double tnow = now.GetSec()+now.GetNanoSec()/1e9;
    unsigned long tm=time(NULL);

    sock->SendRaw("get dist",9);
    printf("Wait data.. \n");
    for (int ii=0; ii<110; ii++) {
      int rc=sock->RecvRaw(&line[ii], 1);
      if (rc<0) { sleep(1); ii=11; continue; }
      if (line[ii]==',') { line[ii]=' ';}
      if (line[ii]=='\n') { line[ii+1]=0; break;}
    }
    //printf("fill: %d %ld y=%f %f\n",i,tm,y0,y2);
    printf("line=%s\n",line);
    float x,y,z,a,d,ox,oy;
    sscanf(line,"%f %f %f %f %f %f %f ",&x,&y,&z,&a,&d,&ox,&oy);
    printf("%ld t=%f x=%f y=%f z=%f a=%f \n",tm,tnow,x,y,z,a);
 
    gr[0]->fillGraph2((double)tnow,x);
    gr[1]->fillGraph2((double)tnow,y);
    gr[2]->fillGraph2((double)tnow,z);
    gr[3]->fillGraph2((double)tnow,a);
    gr[4]->fillGraph2((double)tnow,d);
    gr[5]->fillGraph2((double)tnow,ox);
    gr[6]->fillGraph2((double)tnow,oy);
    gr[7]->fillGraph2(x,z);

       
    for (int k=0; k<Ngraph; k++) {
      c1->cd(k+1);

      gr[k]->graph->Draw("alp");

      gr[k]->graph->GetXaxis()->SetTimeDisplay(1);
      //gr[k]->graph->GetXaxis()->SetTimeFormat("y. %Y %F2000-01-01 00:00:00");
      gr[k]->graph->GetXaxis()->SetTitle("time");
      gr[k]->graph->GetYaxis()->SetTitle("a.u.");
      
      double fsiz=0.065;
      gr[k]->graph->GetXaxis()->SetTitleSize(fsiz);
      gr[k]->graph->GetYaxis()->SetTitleSize(fsiz);
      gr[k]->graph->GetXaxis()->SetLabelSize(fsiz);
      gr[k]->graph->GetYaxis()->SetLabelSize(fsiz);
      gr[k]->graph->GetXaxis()->SetTitleOffset(0.75f);
      gr[k]->graph->GetYaxis()->SetTitleOffset(0.75f);
      
    }
    
    gPad->Update();
    
    c1->Modified();  c1->Update();

    c2->cd();
    gr[7]->graph->Draw("alp");
    gPad->Update();
    c2->Modified();  c2->Update();
 

    gSystem->ProcessEvents();
    usleep(100000);
  }
  return 0;
}
