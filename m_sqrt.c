#include"m_sqrt.h"
#include "common.h"

unsigned int m_sqrt(unsigned int x)
{
uint8 ans=0,p=0x80;
while(p!=0)
{
ans+=p;
if(ans*ans>x)
{
ans-=p;
}
p=(uint8)(p/2);
}
return(ans);
}






