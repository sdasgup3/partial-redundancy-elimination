int func(int a, int b)
{
  int x,y,z, c , d;
    if(x >= 0 ) {
      x = a + b;
      c = a + b;
      y = c;
      d = c + y;
    } else {
      y = a-b;
      d = y;
      c = a - b;
    }
      z = x+y + d + c;

  return z;
}
