static uint8_t search_start_feed(uint8_t byte)
{
  static char line[24];static uint8_t used,collecting;
  if(byte=='@'){used=0;collecting=1;}
  if(!collecting || byte=='\r')return 0;
  if(byte=='\n'){line[used]=0;collecting=0;return strcmp(line,SEARCH_START_FRAME)==0;}
  if(byte<32 || byte>126 || used>=sizeof(line)-1U){used=collecting=0;return 0;}
  line[used++]=(char)byte;return 0;
}
static volatile uint8_t rx_buffer[128],rx_head,rx_tail;
static volatile uint32_t rx_errors,rx_overflows;
void USART1_IRQHandler(void)
{
  uint32_t s=USART1->SR;uint8_t b=(uint8_t)USART1->DR;
  if(s&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){rx_errors++;return;}
  if(s&USART_SR_RXNE){
    uint8_t next=(rx_head+1U)&127U;
    if(next==rx_tail){rx_overflows++;return;}
    rx_buffer[rx_head]=b;rx_head=next;
  }
}
static void search_read_dap(void)
{
  static uint32_t errors,overflows;
  if(errors!=rx_errors || overflows!=rx_overflows){
    errors=rx_errors;overflows=rx_overflows;rx_tail=rx_head;(void)search_start_feed(0);
    if(auto_phase>IDLE && auto_phase<DONE)combined_abort("UART_RX_ERROR");
    return;
  }
  while(rx_tail!=rx_head){
    uint8_t b=rx_buffer[rx_tail];rx_tail=(rx_tail+1U)&127U;
    if(b=='X'||b=='x'){combined_abort("REMOTE_X");(void)search_start_feed(0);}
    else if(search_start_feed(b))start_run();
  }
}
static void search_read_key(uint32_t now)
{
  static uint8_t old,stable,initialized,consumed=1;
  static uint32_t changed;
  uint8_t raw=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET;
  if(!initialized){old=stable=raw;changed=now;initialized=1;return;}
  if(old!=raw){old=raw;changed=now;}
  if(stable==raw || now-changed<25U)return;
  stable=raw;
  if(!stable){
    if(auto_phase>IDLE && auto_phase<DONE){consumed=1;combined_abort("KEY");}
    else consumed=0;
  }else if(consumed)consumed=0;
  else if(auto_phase==IDLE && !reference_run_used)start_run();
}
