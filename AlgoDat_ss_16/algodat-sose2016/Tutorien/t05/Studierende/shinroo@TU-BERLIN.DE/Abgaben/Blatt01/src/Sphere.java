class Sphere {
	
	Point center;
	double radius;
	
	Sphere(int x, int y, int z, double r){
		// TODO
		center = new Point(x,y,z);
		this.radius = r;
	}
	
	Sphere(Point c, double r){
		// TODO
		center = new Point(c.getX(),c.getY(),c.getZ());
		this.radius = r;
	}
	
	double getX(){
		return center.getX(); // TODO
	}

	double getY(){
		return center.getY(); // TODO
	}

	double getZ(){
		return center.getZ(); // TODO
	}


	double getRadius(){
		return this.radius; // TODO
	}

	double calculateDiameter(){
		return (this.radius * 2); // TODO
	}	
	
	double calculatePerimeter(){
		return (2 * Math.PI * this.radius); // TODO
	}
	
	double calculateVolume(){
		return (4.0/3.0) * Math.PI * Math.pow(this.radius,3); // TODO
	}

}

