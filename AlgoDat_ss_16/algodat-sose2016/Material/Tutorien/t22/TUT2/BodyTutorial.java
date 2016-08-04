class BodyTutorial{
    public static void main(String [] args){
        Body b = new Tetrahedron(10.54);
        
        System.out.println(b.get());
        System.out.println("1. Volume = " + b.calculateVolume() + " Surface = " + b.calculateSurface());
        
        b.set(12);
        
        System.out.println("2. Volume = " + b.calculateVolume() + " Surface = " + b.calculateSurface());
    } 
}
