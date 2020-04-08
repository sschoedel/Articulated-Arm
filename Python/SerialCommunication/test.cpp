#include <iostream>

using namespace std;

void increment(int &);

int main()
{
	int h = 4;
	increment(h);
	cout << h << endl;
}

void increment(int& h)
{
	h++;
}