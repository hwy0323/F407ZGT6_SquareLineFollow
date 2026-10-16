/* Successful square controller: 220 follow, 100 ms advance, right pivot
 * until three centred samples after 120 ms, then 100 ms recovery.
 * Only the feature decision is changed for variable-position garages. */
static void search_navigation(uint32_t now)
{
  if(!gray_frames || sample_age_ms(now,gray_last_ms)>200U){combined_abort("GRAY_STALE");return;}
  if(now-run_start_ms>SEARCH_MAX_MS){combined_abort("SEARCH_TIMEOUT");return;}
  odom_update();int32_t progress=-odom_total[0];uint8_t bits=gray_value;
  if(progress< -200){combined_abort("ODOM_DIRECTION");return;}
  if(now-watch_ms>=800U){
    if(progress-watch_position<8){combined_abort("SEARCH_ENCODER_STALL");return;}
    watch_position=progress;watch_ms=now;
  }
  if(state==FOLLOW){
    TrackFeature feature=search_feature(bits,turns,progress);
    if(feature==FEATURE_AMBIGUOUS || feature==FEATURE_END){
      combined_abort("CORNER_WINDOW_OR_FEATURE_AMBIGUOUS");return;
    }
    if(bits==0U){
      feature_hits=0U;
      if(!line_lost){line_lost=1U;line_lost_ms=now;}
      if(now-line_lost_ms>120U){combined_abort("SEARCH_LINE_LOST");return;}
      drive(110,110);return;
    }
    line_lost=0U;
    if(feature==FEATURE_GARAGE || feature==FEATURE_CORNER){
      /* Stop while confirming so a candidate never steers into a bay. */
      stop_motors();
      if(feature_candidate==feature)feature_hits++;else{feature_candidate=feature;feature_hits=1;}
      if(feature_hits<3U)return;
      feature_hits=0U;state_start_ms=now;
      if(feature==FEATURE_GARAGE){
        state=STOPPED;auto_phase=LINE_PAUSE;phase_ms=now;
        char line[120];
        snprintf(line,sizeof(line),"EVENT,GARAGE_LINE,edge=%u,counts=%ld,bits=%02X\r\n",turns,(long)progress,bits);
        send_text(line);
      }else{
        if(turns==3U){combined_abort("ONE_LAP_NO_GARAGE");return;}
        record_corner_event("START",turns+1U,RIGHT_TURN,now);
        turn_direction=RIGHT_TURN;state=ENTER_CORNER;
      }
      return;
    }
    feature_hits=0;feature_candidate=FEATURE_LINE;
    int16_t correction=clamp(line_error(bits)*22,100);
    drive(220+correction,220-correction);
  }else if(state==ENTER_CORNER){
    drive(180,180);
    if(now-state_start_ms>=100U){state=PIVOT;state_start_ms=now;center_hits=0U;}
  }else if(state==PIVOT){
    if(now-state_start_ms>1200U){combined_abort("TURN_TIMEOUT");return;}
    drive(220,-220);
    if(now-state_start_ms>=120U && (bits&0x18U) && active_count(bits)<=2U){
      if(++center_hits>=3U){state=RECOVER;state_start_ms=now;}
    }else center_hits=0;
  }else if(state==RECOVER){
    drive(180,180);
    if(now-state_start_ms>=100U){
      turns++;state=FOLLOW;last_turn_complete_ms=now;
      record_corner_event("DONE",turns,RIGHT_TURN,now);
      feature_candidate=FEATURE_LINE;feature_hits=0;
      watch_ms=now;watch_position=progress;
    }
  }
}
