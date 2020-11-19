#include "extract_point_cloud.h"

rec_3d_data::rec_3d_data(float ry_,float t1_,float t2_,float t3_,float h_,float w_,float l_)
:ry(ry_),t1(t1_),t2(t2_),t3(t3_),h(h_),w(w_),l(l_)
{};

void to_8corners(const rec_3d_data &r)
{
    using namespace boost::numeric::ublas;
    using namespace boost::geometry;
    matrix<double> mref(3,3);
    mref(0,0)=cos(r.ry);
    mref(0,1)=0.0;
    mref(0,2)=sin(r.ry);
    mref(1,0)=0.0;
    mref(1,1)=1.0;
    mref(1,2)=0.0;
    mref(2,0)=-sin(r.ry);
    mref(2,1)=0.0;
    mref(2,2)=cos(r.ry);
  
    matrix<double> corners(3,8);
    double data[]={r.l/2,r.l/2,-r.l/2,-r.l/2,r.l/2,r.l/2,-r.l/2,-r.l/2,
                    0,0,0,0,-r.h,-r.h,-r.h,-r.h,
                    r.w/2,-r.w/2,-r.w/2,r.w/2, r.w/2,-r.w/2,-r.w/2,r.w/2};
    
    std::copy(data,data+24,corners.data().begin());

    matrix<double> rc=prod(mref,corners);
    for (int i=0;i<8;++i)
    {
        rc(0,i)+=r.t1;
        rc(1,i)+=r.t2;
        rc(2,i)+=r.t3;
    }
   std::cout<<rc<<std::endl;
}

extract_point::extract_point(const std::string inf,const std::string outf):pcd_file_(inf),out_file_(outf)
{
   
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    reader.read(pcd_file_,*cloud);

    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::string filenamepath=pcd_file_.substr(0,pcd_file_.find("pcd"));
    std::string lable_file=filenamepath+"txt";
   
    std::reverse(filenamepath.begin(),filenamepath.end());
    std::string filename=filenamepath.substr(1,6);
    std::reverse(filename.begin(),filename.end());

	std::ifstream infile(lable_file.c_str());     

    pcl::CropBox<pcl::PointXYZI> clipper;
    clipper.setInputCloud(cloud);

    //std::chrono::milliseconds sum_time(0);
    std::chrono::duration<double,std::ratio<1,1000>> sum_time(0);

    if (infile.is_open())
    {
        int i=0;
        std::string line;

        while (std::getline(infile, line))
        {

            auto start=std::chrono::system_clock::now();
            std::istringstream iss(line);
            std::string type;
            float truncated;
            float occuluded;
            float  alpha;
            float xmin;
            float ymin;
            float xmax;
            float ymax;
            float h;
            float w;
            float l;
            float x;
            float y;
            float z;
            float rotation_y;
            if (!(iss >> type >> truncated >> occuluded>>alpha>>xmin>>ymin>>xmax>>ymax>>h>>w>>l>>x>>y>>z>>rotation_y)) 
            { 
                break;
            } // error
            else
            {
                if(type=="DontCare")
                {
                    continue;
                }
                else
                {
                outcloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                std::cout <<type <<" "<<truncated <<" "<<occuluded<<" "<<alpha<<" "<<xmin<<" "<<ymin<<" "<<xmax<<" "<<ymax<<" "<<h<<" "<<" "<<w<<" "<<l<<" "<<x<<" "<<y<<" "<<z<<" "<<rotation_y << std::endl;
                //clipper.setTranslation(Eigen::Vector3f(x,y,z));
                clipper.setRotation(Eigen::Vector3f(0,rotation_y,0));
                clipper.setMin(-Eigen::Vector4f(l/2,w/2,0,1.0));
                clipper.setMax(Eigen::Vector4f(l/2,w/2,h,1.0));
                clipper.filter(*outcloud);

                std::chrono::duration<double,std::ratio<1,1000>>  diff=std::chrono::system_clock::now()-start;
                //std::cout<<diff.count()<<std::endl;
                sum_time+= diff;

                std::stringstream  outfilename;
                outfilename<<out_file_<<"/"<<filename<<type<<i<<".pcd";
                    if(!outcloud->empty())
                    {   
                        res_.emplace_back(outcloud);
                        std::cout<<outcloud->size()<<" points writing to "<<outfilename.str()<<std::endl;
                        writer.write<pcl::PointXYZI> (outfilename.str(),*outcloud,false);
                    }
                    else
                    {
                        std::cerr<<"Couldn't find points for "<<type<<i<<std::endl;
                    }
                    
                }  
            }
            i++;
        }
    }
    else
    {
        std::cout << "Unable to open file" <<std::endl;
    }
       dur_=sum_time;
};

std::chrono::duration<double,std::ratio<1,1000>> extract_point::extract_time()
{

    return dur_;
}
