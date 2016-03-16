library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adder is
	generic( WIDTH : integer := 32 );
	port ( a,b  : in  std_logic_vector( WIDTH-1 downto 0 );
	       sum  : out std_logic_vector( WIDTH-1 downto 0 );
	       cout : out std_logic);
end entity;

architecture behavioral of adder is
	signal cinarr : std_logic_vector( WIDTH-1 downto 0 );
	component fulladd is
		port (a, b, cin: in std_logic;
		     sum, cout: out std_logic);
	end component;
begin

	--instaziere fulladd komponent
	

	-- Addierer-Beschreibung hier einfuegen
	cinarr(0) <= '0';

	adders: for N in 0 to WIDTH-1 generate

		myfulladder: entity work.fulladd(behavioral)
			port map(
				a => a(N),
				b => b(N),
				cin => 	cinarr(N),
				cout => cinarr(N+1),
				sum => sum(N)
			);

	end generate adders;

end architecture;
