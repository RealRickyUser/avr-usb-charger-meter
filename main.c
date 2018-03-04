#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nlcd.h"

#define F_CPU 1000000UL
#define TIMER_DIV F_CPU / 1000 / 64

#define CALIBRATE 1
#define VCC 2
#define ICC 3

unsigned long timer;
unsigned long timer_old;
unsigned int cal_value;
unsigned int vcc_value;
unsigned int i_value;
float multipler = 2.5;
float R = 0.125;
float I_mul = 9.3;//1 + 46.2/5.56k;
unsigned char postfix[] = " mah";
unsigned char hmask[] = "00:00:00";
float i_total;
float i_step_val;
unsigned char i_step_cnt;

unsigned char order = 0;
struct lcd_nokia lcd;

void floatToChar(char* buf,  unsigned char cnt, float f);
void longToChar(char* buf,  unsigned char cnt, unsigned long f);

unsigned char getNum(unsigned char c) {
	return '0' + c;
}

void clearMem(char* buffer, unsigned char size) {
	for(unsigned char x = 0; x < size;x++) {
		buffer[x] = 0;
	}
}

unsigned char getMux(unsigned char port) {
	return (1 << 4) | (1 << REFS0) | (0 << REFS1) | port;
}

void writeTime(char *buf) {
	unsigned long t = timer;
	unsigned char h = t / 3600;
	t = t - h * 3600;
	unsigned char m = t / 60;
	unsigned char s = t - m * 60;
	unsigned char pos = 0;
	while (pos < 8) {
		buf[pos] = hmask[pos];
		pos++;
	}
	unsigned char c;
	c = h / 10;
	buf[0] = getNum(c);
	buf[1] = getNum(h - c * 10);
	c = m / 10;
	buf[3] = getNum(c);
	buf[4] = getNum(m - c * 10);
	c = s / 10;
	buf[6] = getNum(c);
	buf[7] = getNum(s - c * 10);
}

void print() {
	float val = vcc_value * 1.5 * multipler / cal_value;
	char buf2[10];
	clearMem(buf2, 10);
	floatToChar(buf2, 5, val);
	buf2[5] = 'V';
	lcd_nokia_set_cursor(&lcd, 0, 0);
	lcd_nokia_write_string(&lcd, buf2, 1);
	val = i_value * 1.5 / (cal_value * I_mul * R);
	clearMem(buf2, 10);
	floatToChar(buf2, 5, val);
	buf2[5] = 'A';
	lcd_nokia_set_cursor(&lcd, 0, 10);
	lcd_nokia_write_string(&lcd, buf2, 1);

	unsigned long now = timer;
	unsigned long delta_time = now - timer_old;
	if (delta_time == 0) {
		i_step_cnt++;
		i_step_val += val;
	} else {
		timer_old = timer;
		//save I for avg
		i_total += i_step_val * delta_time / (i_step_cnt == 0 ? 1 : i_step_cnt);
		i_step_val = 0;
		i_step_cnt = 0;
	}
	clearMem(buf2, 10);
	unsigned long i = (unsigned long) (i_total * 1000 / 3600);
	longToChar(buf2, 5, i);
	unsigned char pos = 0, offset;
	while (pos < 10 &&  buf2[pos] != 0) {
		pos++;
	}
	offset = pos;
	while (pos < 10 || pos - offset < 4) {
		buf2[pos] = postfix[pos - offset];
		pos++;
	}
	lcd_nokia_set_cursor(&lcd, 0, 20);
	lcd_nokia_write_string(&lcd, buf2, 1);
	clearMem(buf2, 10);
	writeTime(buf2);
	lcd_nokia_set_cursor(&lcd, 0, 30);
	lcd_nokia_write_string(&lcd, buf2, 1);
	lcd_nokia_render(&lcd);
}

ISR (ADC_vect)//прерывание по завершению преобразования АЦП
{
	char buf2[10];
	clearMem(buf2, 10);
	switch (order) {
		case CALIBRATE:
			cal_value = ADCW;
			ADMUX = getMux(PC1);
			//lcd_nokia_set_cursor(&lcd, 0, 0);
			order = VCC;
			break;
		case VCC:
			vcc_value = ADCW;
			ADMUX = getMux(PC2);
			//print();
			//lcd_nokia_set_cursor(&lcd, 0, 10);
			order = ICC;
			break;
		case ICC:
			i_value = ADCW;
			ADMUX = getMux(PC0);
			print();
			//lcd_nokia_set_cursor(&lcd, 0, 20);
			//_delay_ms(100);
			order = CALIBRATE;
			break;
	}
	//floatToChar(buf2, 5, ADCW);
	//lcd_nokia_write_string(&lcd, buf2, 1);
	//lcd_nokia_render(&lcd);
	//блок операторов ниже реализует "светящийся столбик" в зависимости от измеренной величины напряжения
	ADCSRA |= (1<<ADSC);//запускаем очередное преобразование
}

ISR(TIMER1_COMPA_vect) {
    timer++;
	/*char buf2[10];
	clearMem(buf2, 10);
	unsigned long t = timer;
	longToChar(buf2, 5, t);
	lcd_nokia_set_cursor(&lcd, 0, 30);
	lcd_nokia_write_string(&lcd, buf2, 1);
	lcd_nokia_render(&lcd);*/
}


void floatToChar(char* buf,  unsigned char cnt, float f) {
	unsigned char digs = 1, lessOne = 0;
	float g = f;
	while (g >= 10) {
		g /= 10;
		digs++;
	}
	while (g < 1) {
		g *= 10;
		digs++;
		lessOne = 1;
	}
	unsigned char pos = 0;
	while(pos < cnt) {
		if (lessOne == 1) {
			if (pos == 0) {
				buf[pos] = getNum(0);
			} else if (pos == 1) {
				buf[pos] = '.';
			} else if (pos < digs) {
				buf[pos] = getNum(0);
			} else {
				unsigned char o = (unsigned char)g;
				buf[pos] = getNum(o);
				g = (g - o) * 10;
			}
			pos++;
			continue;
		}
		if (pos == digs) {
			buf[pos] = '.';
			pos++;
			continue;
		}
		unsigned char o = (unsigned char)g;
		buf[pos] = getNum(o);
		pos++;
		g = (g - o) * 10;
		if (pos > digs && g == 0) {
			break;
		}
	}
}


void longToChar(char* buf,  unsigned char cnt, unsigned long f) {
	unsigned long val = f, dec;
	unsigned char dig, pos = 0, a, b;
	if (f == 0) {
		buf[0] = '0';
		return;
	}
	while (pos < cnt) {
		dec = (unsigned long) (val / 10);
		dig = val - dec*10;
		buf[pos] = getNum(dig);
		val = val / 10;
		if (val == 0) {
			break;
		}
		pos++;
	}
	//rotate chars
	for (unsigned char x = 0; x < pos; x++) {
		a = buf[x];
		b = buf[pos - x];
		buf[pos - x] = a;
		buf[x] = b;
	}

}

int main() {
	//struct lcd_nokia lcd;
	timer = 0;
	timer_old = 0;
	i_step_val = 0;
	i_step_cnt = 0;

	lcd_nokia_init();
	lcd_nokia_clear(&lcd);
	i_total = 0;
	/*float fl = 0.00035f;
	char val[10];
	floatToChar(val, 7, fl);
	lcd_nokia_write_string(&lcd, val,1);
	clearMem(val, 10);
	floatToChar(val, 7, 0.25f);*/
	//lcd_nokia_set_cursor(&lcd, 0, 10);
	//lcd_nokia_write_string(&lcd, val,1);
	//lcd_nokia_set_cursor(&lcd, 25, 10);
	//lcd_nokia_write_string(&lcd, "6643 mah",1);
//	lcd_nokia_set_cursor(&lcd, 0, 10);
//	lcd_nokia_write_string(&lcd, "Nice!", 3);
	//lcd_nokia_render(&lcd);

		/*for (;;) {
			_delay_ms(1000);
	}*/

	DDRB = 0xFF;
	DDRD = 0xFF;
	DDRC = 0;
	order = CALIBRATE;
	ADMUX = getMux(PC0);
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	/*timer*/
   TCCR1B |= (1 << WGM12); // configure timer1 for CTC mode
   TIMSK |= (1 << OCIE1A); // enable the CTC interrupt
   /*OCR1A   = 125; // set the CTC compare value. every 1ms
   TCCR1B |= ((0 << CS10) | (1 << CS11)); // start the timer at 1MHz/8*/
   OCR1A   = 15625; // set the CTC compare value. every 1ms
   TCCR1B |= ((1 << CS10) | (1 << CS11)); // start the timer at 1MHz/64


	sei();
	while(1 == 1) {
		//writeall(12.3f);
		//display_off();
		//_delay_ms(1000);
		//writeall(1.4f);
	}
}
