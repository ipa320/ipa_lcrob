
static void int2str(char *str, u16 val) {
	u16 div=10000;
	while(!(val/div) && div>1) div/=10;
	while(div) {
		*str = '0'+(val/div);
		str++;
		val %=div;
		div/=10;
	}
	*str = '\n';
	str++;
	*str = 0;
}

#define ENABLE   "EN\n"
#define DISABLE  "DI\n"
#define POSITION "POS\n"

void init() {
	send(ENABLE, 0);
	send(ENABLE, 1);
}

void stop() {
	send(DISABLE, 0);
	send(DISABLE, 1);
}

void setVel(u16 rpm1, u16 rpm2) {
	char buffer[8];
	buffer[0]='V';

	int2str(buffer+1, rpm1);
	send(buffer, 0);

	int2str(buffer+1, rpm2);
	send(buffer, 1);
}

i32 getPos(u08 motor) {
	char c;
	i32 r=0;

	send(POSITION, motor);
	while( (c=recv(motor))!='\r' ) {
		if(c=='-') r*=-1;
		else if(c>='0'&&c<='9') {
			r*=10;
			r+=c-'0';
		}
	}

	return r;
}
