#include <extract_point_cloud.h>

int main()
{
extract_point ep("../data/000001.pcd","../output");
auto t=ep.extract_time();
std::cout<<"Time: "<< t.count()<<std::endl;
return 0;
}