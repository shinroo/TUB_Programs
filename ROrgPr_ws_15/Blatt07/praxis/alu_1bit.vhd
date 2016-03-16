library ieee;
use ieee.std_logic_1164.all;

entity alu_1bit is
	generic(carrySelect : boolean := false);
	port(operation : in std_logic_vector(1 downto 0);
		 a : in std_logic;
		 aInvert : in std_logic;
		 b : in std_logic;
		 bInvert : in std_logic;
		 carryIn : in std_logic;
		 less : in std_logic;
		 result : out std_logic;
		 carryOut : out std_logic;
		 set : out std_logic);
end alu_1bit;

architecture structural of alu_1bit is

component mux2 is
	port( a: in  std_logic;
	      sel        : in  std_logic;
	      y          : out std_logic);
end component mux2;

component mux4 is
	port( a, b, c, d : in  std_logic;
	      sel        : in  std_logic_vector(1 downto 0);
	      y          : out std_logic);
end component mux4;

component adder_1bit is
	port(a : in std_logic;
		 b : in std_logic;
		 cin : in std_logic;
		 sum : out std_logic;
		 cout : out std_logic);
end component adder_1bit;

signal a_in, b_in, aAb, aOb, sum: std_logic;

begin

	set <= sum;
	aAb <= a_in and b_in;
	aOb <= a_in or b_in;

aInv:	entity work.mux2 (behavioral)
	  port map ( 	a => a,
			sel => aInvert,
			y => a_in);

bInv:	entity work.mux2(behavioral)
	  port map( 	a => b,
			sel => bInvert,
			y => b_in);

add:	entity work.adder_1bit(behavioral)
	  port map(	a => a_in,
		 	b => b_in,
		 	cin => carryIn,
		 	sum => sum,
			cout => carryOut);

big_mux: entity work.mux4(behavioral)
	  port map(	a => aAb,
			b => aOb,
			c => sum,
			d => less,
			sel => operation,
			y => result);

end structural;
