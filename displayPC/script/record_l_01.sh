#include <iostream>
class CUI
{
public:
	virtual ~CUI(){}
	virtual void OnPaint()
	{
		std::cout<< "Paint" <<std::endl;
	}
};

class CNormalUI : public CUI
{
public:
	virtual void OnPaint()
	{
		std::cout<< "Normal Paint" <<std::endl;
	}
};

class CColorUI : public CUI
{
public:
	virtual void OnPaint()
	{
		std::cout<< "Colorfullly Paint" <<std::endl;
	}
};

int main(int argc, char **argv)
{
	CUI* p = new CNormalUI();
	p->OnPaint(); // Normal Paint
	delete p;

	p = new CColorUI();
	p->OnPaint(); // Colorfullly Paint
	delete p;
	
	return 0;
}