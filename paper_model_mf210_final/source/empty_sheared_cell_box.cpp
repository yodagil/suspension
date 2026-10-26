// Conservative stable-cell variant. In addition to bad-node exclusion,
// avoid bounding boxes of projected inverse-sheared unsafe physical cells.
// Each accepted physical cell has all eight corners classified stable.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>
using namespace std;

int floordiv(int a,int b) { int q=a/b; if(a%b<0)--q; return q; }
vector<int> incident(int num,int den) {
  int f=floordiv(num,den);
  if(num%den==0)return {f-1,f};
  return {f};
}
int main(int argc,char**argv){
  if(argc!=2)return 2;
  ifstream in(argv[1]); int nv,nr,np; double av,ar,ap,bv,bp;
  in>>nv>>nr>>np>>av>>ar>>ap>>bv>>bp;
  vector<int> m(nv*nr*np); for(auto &x:m)in>>x;
  if(!in)return 3;
  auto good=[&](int i,int j,int k){return m[(i*nr+j)*np+k]!=0;};
  long long best=-1,nties=0,configs=0,rectangles=0;
  array<int,8> bounds{};
  int minroll=100000,maxroll=0;
  for(int r0=0;r0<nr-1;r0++){
    if(r0>ar+1e-10)continue;
    for(int r1=r0+1;r1<nr;r1++){
      if(r1<ar-1e-10)continue;
      int L=r1-r0;
      set<int> dvs={0,(int)floor(bv*L),(int)ceil(bv*L)};
      set<int> dps={0,(int)floor(bp*L),(int)ceil(bp*L)};
      for(int sv:dvs)for(int sp:dps){
        int vl=max(0,-sv),vh=min(nv-1,nv-1-sv);
        int pl=max(0,-sp),ph=min(np-1,np-1-sp);
        int V=vh-vl,P=ph-pl;
        if(V<=0||P<=0)continue;
        double qav=av-sv*(ar-r0)/L,qap=ap-sp*(ar-r0)/L;
        if(qav<vl-1e-10||qav>vh+1e-10||qap<pl-1e-10||qap>ph+1e-10)continue;
        configs++;
        vector<unsigned char> bad(V*P,0);
        for(int r=r0;r<=r1;r++){
          vector<vector<int>> vi(nv),pi(np);
          for(int v=0;v<nv;v++)vi[v]=incident(v*L-sv*(r-r0),L);
          for(int p=0;p<np;p++)pi[p]=incident(p*L-sp*(r-r0),L);
          for(int v=0;v<nv;v++)for(int p=0;p<np;p++){
            if(good(v,r,p))continue;
            for(int i:vi[v])if(i>=vl&&i<vh)
              for(int k:pi[p])if(k>=pl&&k<ph)bad[(i-vl)*P+k-pl]=1;
          }
        }
        // Closed-point exclusion alone misses instability between coarse
        // vc nodes. Require the shape to lie within fully stable-corner
        // physical cells, using a conservative projected-bounding-box test.
        // Open cell interiors: integer-aligned safe boxes are not eroded
        // merely because an unsafe cell touches an external boundary.
        for(int r=r0;r<r1;r++)for(int v=0;v<nv-1;v++)for(int p=0;p<np-1;p++){
          bool safe=true;
          for(int dv=0;dv<2;dv++)for(int dr=0;dr<2;dr++)for(int dp=0;dp<2;dp++)
            safe=safe&&good(v+dv,r+dr,p+dp);
          if(safe)continue;
          int sv0=sv*(r-r0),sv1=sv*(r+1-r0);
          int sp0=sp*(r-r0),sp1=sp*(r+1-r0);
          int a=max(vl,floordiv(v*L-max(sv0,sv1),L));
          int b=min(vh-1,-floordiv(-((v+1)*L-min(sv0,sv1)),L)-1);
          int c=max(pl,floordiv(p*L-max(sp0,sp1),L));
          int d=min(ph-1,-floordiv(-((p+1)*L-min(sp0,sp1)),L)-1);
          for(int i=a;i<=b;i++)for(int k=c;k<=d;k++)bad[(i-vl)*P+k-pl]=1;
        }
        vector<int> heights(P,0),starts(P),hs(P);
        for(int row=0;row<V;row++){
          for(int p=0;p<P;p++)heights[p]=bad[row*P+p]?0:heights[p]+1;
          int top=0;
          for(int end=0;end<=P;end++){
            int height=end<P?heights[end]:0,start=end;
            while(top>0&&hs[top-1]>height){
              --top;int p0=starts[top],h=hs[top];start=p0;
              int v0=vl+row-h+1,v1=vl+row+1,p1=pl+end;p0+=pl;
              if(!(v0<=qav+1e-10&&v1>=qav-1e-10&&p0<=qap+1e-10&&p1>=qap-1e-10))continue;
              rectangles++;
              long long volume=1LL*(v1-v0)*(p1-p0)*L;
              array<int,8> b={v0,v1,r0,r1,p0,p1,sv,sp};
              int roll=p1-p0+abs(sp);
              if(volume>best){best=volume;bounds=b;nties=1;minroll=maxroll=roll;}
              else if(volume==best){nties++;if(b<bounds)bounds=b;minroll=min(minroll,roll);maxroll=max(maxroll,roll);}
            }
            if(height>0&&(top==0||hs[top-1]<height)){starts[top]=start;hs[top]=height;top++;}
          }
        }
      }
    }
  }
  cout<<"{\"volume_grid_units\":"<<best<<",\"bounds\":[";
  for(int i=0;i<8;i++){if(i)cout<<",";cout<<bounds[i];}
  cout<<"],\"ties\":"<<nties<<",\"tie_roll_span_grid_min\":"<<minroll
      <<",\"tie_roll_span_grid_max\":"<<maxroll<<",\"configurations\":"<<configs
      <<",\"rectangles\":"<<rectangles<<"}\n";
}
