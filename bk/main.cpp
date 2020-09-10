#include<iostream>
#include<fstream>
#include<ostream>
using namespace std;

int main(){
	ofstream fn_;
	fn_.open("test.csv",ios::out);
	for(int i=0;i<10;i++){
            for(int j=0;j<10;j++){
		fn_<<i*j<<",";	
		}
	    fn_<<endl;
	}
	fn_.close();
	while(1)
	cout<<"here";
	return 0;
}
