/* Only this complete frame or a deliberate KEY release can start once. */
static uint8_t combined_start_feed(uint8_t byte)
{
  static char line[32];
  static uint8_t used,collecting;
  if(byte=='@'){used=0U;collecting=1U;}
  if(!collecting || byte=='\r')return 0U;
  if(byte=='\n'){
    line[used]=0;collecting=0U;
    return strcmp(line,ENTRY_START_FRAME)==0;
  }
  if(byte<32U || byte>126U || used>=sizeof(line)-1U){
    collecting=used=0U;return 0U;
  }
  line[used++]=(char)byte;
  return 0U;
}

static void combined_abort(const char *reason)
{
  char line[120];
  reference_run_used=1U; /* A stop/cancel cannot arm a later release/noisy G. */
  if(pivot_active)reference_finish(reason);
  if(state!=STOPPED)stop_run(reason);
  reference_halted=1U;
  auto_phase=4U;
  stop_motors();
  (void)snprintf(line,sizeof(line),"EVENT,TASK4_ABORT,%s,%s\r\n",ENTRY_EDGE,reason);
  send_text(line);
}

static void combined_tick(uint32_t now)
{
  if(auto_phase==2U && now-auto_pause_start_ms>=ENTRY_WAIT_MS){
    if(!entry_confirmed || turns!=ENTRY_REQUIRED_CORNERS ||
       parking_events!=ENTRY_PASS_LINES){
      combined_abort("WRONG_EDGE_OR_GARAGE_SEQUENCE");return;
    }
    auto_phase=3U;
    send_text("EVENT,TASK4_ENTRY_BEGIN," ENTRY_EDGE ",UNTESTED\r\n");
    reference_start(now);
    if(!pivot_active)combined_abort("FIRST_STEP_REFUSED");
    return;
  }
  if(auto_phase!=3U)return;
  if(reference_halted){combined_abort("REFERENCE_STEP_FAILED");return;}
  if(pivot_active)return;
  if(reference_next==ENTRY_STEP_COUNT){
    stop_motors();auto_phase=4U;
    send_text("EVENT,TASK4_ENTRY_DONE," ENTRY_EDGE ",UNTESTED,motors_off\r\n");
    return;
  }
  if(now-pivot_last_stop_ms>=150U){
    reference_start(now);
    if(!pivot_active)combined_abort("NEXT_STEP_REFUSED");
  }
}

static volatile uint8_t rx_buffer[128],rx_head,rx_tail;
static volatile uint32_t rx_errors,rx_overflows;
void USART1_IRQHandler(void)
{
  uint32_t status=USART1->SR;
  uint8_t byte=(uint8_t)USART1->DR;
  if(status&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){rx_errors++;return;}
  if(status&USART_SR_RXNE){
    uint8_t next=(uint8_t)((rx_head+1U)&127U);
    if(next==rx_tail){rx_overflows++;return;}
    rx_buffer[rx_head]=byte;rx_head=next;
  }
}
static void entry_read_dap(void)
{
  static uint32_t previous_errors,previous_overflows;
  if(rx_errors!=previous_errors || rx_overflows!=previous_overflows){
    previous_errors=rx_errors;previous_overflows=rx_overflows;
    (void)combined_start_feed(0U);
    rx_tail=rx_head;
    if(auto_phase>0U && auto_phase<4U)combined_abort("UART_RX_ERROR");
    return;
  }
  while(rx_tail!=rx_head){
    uint8_t b=rx_buffer[rx_tail];rx_tail=(uint8_t)((rx_tail+1U)&127U);
    if(b=='X' || b=='x'){combined_abort("REMOTE_X");(void)combined_start_feed(0U);}
    else if(combined_start_feed(b)){
      if(auto_phase==0U && !reference_run_used)start_run();
      else send_text("EVENT,REFUSE_START,once_per_reset_or_busy\r\n");
    }
  }
}

static void entry_read_key(uint32_t now)
{
  static uint8_t old_raw=1U,stable=1U,stop_consumed=1U;
  static uint8_t initialized;
  static uint32_t change_ms;
  uint8_t raw=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET;
  if(!initialized){old_raw=stable=raw;change_ms=now;initialized=1U;return;}
  if(raw!=old_raw){old_raw=raw;change_ms=now;}
  if(raw==stable || now-change_ms<25U)return;
  stable=raw;
  if(!stable){
    if(auto_phase>0U && auto_phase<4U){stop_consumed=1U;combined_abort("KEY");}
    else stop_consumed=0U;
  }else if(stop_consumed)stop_consumed=0U;
  else if(auto_phase==0U && !reference_run_used)start_run();
}
