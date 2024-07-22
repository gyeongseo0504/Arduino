void setup() 
{
  size(600,450);
  background(255);  
  for (int i=0; i<4; i++)
  {
    house1(i*150, 100);
    house2(i*150, 300);    
  }
}

void house1(int x, int y) 
{
  fill(0);
  triangle(x, y, x+100, y, x+50, y-30);
  fill(100);
  rect(x, y, 100, 100);
}
void house2(int x, int y)
{
  pushMatrix();
  println(x, y);
  translate(x, y);
  fill(200, 50, 0);
  triangle(0, 0, 100, 0, 50, -30);
  fill(50, 100, 200);
  rect(0, 0, 100, 100);
  popMatrix();
}
