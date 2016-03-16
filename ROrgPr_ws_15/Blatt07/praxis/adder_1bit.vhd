library ieee;
use ieee.std_logic_1164.all;

entity adder_1bit is
	port(a : in std_logic;
		 b : in std_logic;
		 cin : in std_logic;
		 sum : out std_logic;
		 cout : out std_logic);
end adder_1bit;

architecture behavioral of adder_1bit is
begin

	sum <= a xor b xor cin;
	cout <= (a and b) or (a and cin) or (b and cin);

end behavioral;

architecture carrySelect of adder_1bit is

	signal sum1, sum2, cout1, cout2 : std_logic;
	signal cin1 : std_logic := '0';
	signal cin2 : std_logic := '1';

	component adder_1bit is
		port(a : in std_logic;
			 b : in std_logic;
			 cin : in std_logic;
			 sum : out std_logic;
			 cout : out std_logic);
	end component adder_1bit;


begin

adder1: entity work.adder_1bit(behavioral)
		port map (a => a,
							b => b,
							cin => '0',
							cout => cout1,
							sum => sum1);

adder2: entity work.adder_1bit(behavioral)
			port map (a => a,
								b => b,
								cin => '1',
								cout => cout2,
								sum => sum2);

		sum	<= sum1 when cin = '0' else
					 sum2 when cin = '1' else 'X';

		cout <= cout1 when cin = '0' else
						cout2 when cin = '1' else 'X';

end carrySelect;
