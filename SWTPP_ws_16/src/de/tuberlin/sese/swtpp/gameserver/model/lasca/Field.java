package de.tuberlin.sese.swtpp.gameserver.model.lasca;

public class Field {
	public int x, y;
	public String stones;

	public Field(int x, int y){
		this.x = x;
		this.y = y;
		this.stones = new String("");
	}

	public boolean isOfficer(){
		return !stones.substring(0, 1).equals(stones.substring(0, 1).toLowerCase());
	}
	
	public boolean isWhite(){
		return stones.substring(0, 1).toLowerCase().equals("w");
	}

	public boolean isBlack(){
		return !isWhite();
	}
	
	public boolean isFree(){
		return stones.equals("");
	}
	
	public String toString(){
		return getFieldNotation(this);
	}
	

	private static String getFieldNotation(Field f) throws RuntimeException {
		if (f.x == 0)
			return new String("a" + (f.y+1));
		else if (f.x == 1)
			return new String("b" + (f.y+1));
		else if (f.x == 2)
			return new String("c" + (f.y+1));
		else if (f.x == 3)
			return new String("d" + (f.y+1));
		else if (f.x == 4)
			return new String("e" + (f.y+1));
		else if (f.x == 5)
			return new String("f" + (f.y+1));
		else if (f.x == 6)
			return new String("g" + (f.y+1));
		else
			throw new RuntimeException("Invalid field!");
	}
}
