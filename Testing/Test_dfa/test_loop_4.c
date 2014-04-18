void func(int a, int b, int c)
{
  int x,y,z,  d;
L1:  
  if(b>c) {
    x  = a + b;
    x = x+ 1;
    goto L1;
  } else {
    goto L2;
  }

L2:  
  return;
}
