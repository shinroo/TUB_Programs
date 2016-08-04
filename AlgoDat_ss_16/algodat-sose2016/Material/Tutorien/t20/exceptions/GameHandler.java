package exceptionGame;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Random;

public class GameHandler {
	
	private int maxnumber;
	private int winningNumber; 
	
	public GameHandler( int max ){
		this.maxnumber = max; 
	}
	
	public void initGame() {
	    Random randomGenerator = new Random();
		this.winningNumber = randomGenerator.nextInt(maxnumber);
		System.out.println("Hello, Welcome: Guess a Number between 0 and " + this.maxnumber);
	}
	
	public boolean readInGuess ()
			{
	    InputStreamReader isr = new InputStreamReader(System.in);
	    BufferedReader br = new BufferedReader(isr);
	    String input = null;
		try {
			input = br.readLine();
		} catch (IOException e) {
			e.printStackTrace();
		}
	    int guessed =  Integer.parseInt(input);
	    if (guessed == winningNumber){
	    	System.out.println("HURRA");
	    	return true; 
	    }
	    else{
	    	return false;
	    }
	}

	public void penalty(int volt) {
		System.out.print("SHOCK  ");
    	for(int i = 0; i < volt; i++)
    		System.err.print("/|");
    	System.out.print("\n");	
	}
}
