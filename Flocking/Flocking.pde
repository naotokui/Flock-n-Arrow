import processing.opengl.*;
import processing.pdf.*;

int numBoids	= 15848;
int numSteps	= 45;
int drawCount	= 0;
Flock flock;

color leftColor		= color(255, 0, 0);
color rightColor	= color(0, 0, 255);

boolean record		= false;

PImage arrowimg;

void setup() {
	size(1024, 1024, OPENGL);
	
	// Base image
	arrowimg = loadImage("arrow7.png");
	arrowimg.loadPixels();
	
	// Smoothing
	smooth();
	
	// Init
	reset();
}

void draw() {
	// if(keyPressed) {
	//     	if (keyCode == DOWN || key == 'R') {
	//       		record = true;
	//     	}
	//   	}
	if (drawCount >= 20 && drawCount % 10 == 0) record	 = true;
	
	if (record) {
		println("exporting PDF...");
		String pdfname		= "arrow-" + hour() + "-" + minute()+ "-" + second() + ".pdf";
		beginRaw(PDF, pdfname); 
	}
	background(255);
	fill(255);
	rect(0,0,width,height);
	flock.run();
	drawCount++;
	println("step: " + drawCount);
	if (record) {
		endRaw();
		record = false;
		println("done!");
	}
}

// Reset flocking
// void keyPressed() {
// 	if (key=='r') record = true;
// }

void reset(){
	
	// Flock object
	flock = new Flock();
		
	float coef	= 1.0;
	float zoom 	= width/(float)arrowimg.width * coef;
	
	for (int i = 0; i < numBoids; i++) {
		while(true){
			int x 		= (int)random(width);
			int y		= (int)random(height);

			int ix		= (int)max(0, min((x-arrowimg.width * 0.0)/zoom, arrowimg.width-1)); 	// centering
			int iy		= (int)max(0, min((y-arrowimg.height*0.0)/zoom, arrowimg.height-1));  	// vertical centering

			if (brightness(arrowimg.pixels[ix + arrowimg.width * iy])<brightness(color(100))){
				flock.addBoid(new Boid(new PVector(x, y), random(4.0)+1.0, 0.05));		
				break;
			}
		}
	}
}
