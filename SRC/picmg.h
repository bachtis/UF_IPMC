/*
    UF_IPMC/picmg.h

    Original work Copyright (C) 2007-2008 Gokhan Sozmen
    http://www.coreipm.com

    Modified work Copyright 2020 Aleksei Greshilov
    aleksei.greshilov@cern.ch

    UF_IPMC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    UF_IPMC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with UF_IPMC.  If not, see <https://www.gnu.org/licenses/>.
*/
extern FRU_INFO fru[MAX_FRU_DEV_ID + 1];

void picmg_process_command( IPMI_PKT *pkt );
void picmg_get_address_info( IPMI_PKT *pkt );
void picmg_get_shelf_address_info( IPMI_PKT *pkt );
void picmg_set_shelf_address_info( IPMI_PKT *pkt );
void picmg_fru_control( IPMI_PKT *pkt );
void picmg_get_fru_led_properties( IPMI_PKT *pkt );
void picmg_get_led_color_capabilities( IPMI_PKT *pkt );
void picmg_set_fru_led_state( IPMI_PKT *pkt );
void picmg_get_fru_led_state( IPMI_PKT *pkt );
void picmg_set_ipmb_state( IPMI_PKT *pkt );
void picmg_set_fru_activation( IPMI_PKT *pkt );
void picmg_set_port_state( IPMI_PKT *pkt );
void picmg_get_port_state( IPMI_PKT *pkt );
void picmg_compute_power_properties( IPMI_PKT *pkt );
void picmg_set_power_level( IPMI_PKT *pkt );
void picmg_get_power_level( IPMI_PKT *pkt );
void picmg_renegotiate_power( IPMI_PKT *pkt );
void picmg_get_fan_speed_properties( IPMI_PKT *pkt );
void picmg_set_fan_level( IPMI_PKT *pkt );
void picmg_get_fan_level( IPMI_PKT *pkt );
void picmg_bused_resource( IPMI_PKT *pkt );
void picmg_get_ipmb_link_info( IPMI_PKT *pkt );
void picmg_get_device_locator_rec_id( IPMI_PKT *pkt );
void picmg_get_picmg_properties( IPMI_PKT *pkt );
void picmg_set_fru_activation_policy( IPMI_PKT *pkt );
void picmg_get_fru_activation_policy( IPMI_PKT *pkt );
unsigned char picmg_get_hw_address( void );
unsigned char picmg_get_ipmb0_address( void );
void picmg_m2_state( unsigned fru );
void picmg_m2_to_m3_state( unsigned fru );
void picmg_handle_switch_check( void );

