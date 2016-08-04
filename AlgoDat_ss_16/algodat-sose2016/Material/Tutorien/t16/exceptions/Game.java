package exceptionGame;

public class Game {

	private int range;
	private int voltage;
	private GameHandler referee;
	
	public Game(int range){
		this.range = range; 
	}
	
	public void playTheGame ( ){

		this.referee =  new GameHandler(this.range);
		this.voltage = 1; 
		boolean currentGuess = false; 
		
		referee.initGame();
		for (int j = 0; j< this.range; j++ ){
			try {
				currentGuess = referee.readInGuess();
				if (currentGuess)
					break;
				else {
					System.out.println( "Ah... NO!" );
					referee.penalty(this.voltage);
				}
			} catch  (Exception e) {
				System.out.println("Please enter a Integer : voltage doubled");
				this.voltage *= 2; 
			}
			
		}
		System.out.println("done");
	
	}
}
