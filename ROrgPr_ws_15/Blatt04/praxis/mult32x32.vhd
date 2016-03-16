library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity mult32x32 is
    port(a : in std_logic_vector(31 downto 0);
         b : in std_logic_vector(31 downto 0);
         y : out std_logic_vector(63 downto 0));
end mult32x32;


architecture structural of mult32x32 is
signal a0, a1, b0, b1 : std_logic_vector(15 downto 0);
signal mult1, mult3, mult4 : signed(31 downto 0);
signal mult2 : unsigned(31 downto 0);
signal conc, toAdd64: std_logic_vector(63 downto 0);
signal added33 : signed(32 downto 0);
begin
    a0 <= a(15 downto 0);
    a1 <= a(31 downto 16);
    b0 <= b(15 downto 0);
    b1 <= b(31 downto 16);

    mult16x16_01: entity work.mult16x16SignedSigned (behavioral)
		port map(a => signed(a1),
			b => signed(b1),
			y => mult1);

    mult16x16_02: entity work.mult16x16UnsignedUnsigned (behavioral)
		port map(a => unsigned(a0),
			b => unsigned(b0),
			y => mult2);

    mult16x16_03: entity work.mult16x16UnsignedSigned (behavioral)
		port map(a => unsigned(b0),
			b => signed(a1),
			y => mult3);

    mult16x16_04: entity work.mult16x16UnsignedSigned (behavioral)
		port map(a => unsigned(a0),
			b => signed(b1),
			y => mult4);

    conc <= std_logic_vector(mult1) & std_logic_vector(mult2);

    add33_33: entity work.add33 (behavioral)
		port map(a => mult3,
			b => mult4,
			y => added33);

    
    process (added33)
    variable component1, component3 : std_logic_vector(15 downto 0);
    variable component2 : std_logic_vector(31 downto 0);
    variable component0 : std_logic_vector(32 downto 0);
    begin
	component0 := std_logic_vector(added33);
	component1 := (others => component0(32));
	component2 := component0(31 downto 0);
	component3 := (others => '0');
	toAdd64 <= component1 & component2 & component3;
    end process;

    y <= std_logic_vector(signed(conc) + signed(toAdd64));
    

end structural;
