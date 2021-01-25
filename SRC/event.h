/*
    UF_IPMC/event.h

    Original work Copyright (C) 2007-2008 Gokhan Sozmen
    http://www.coreipm.com

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

void ipmi_get_pef_capabilities( IPMI_PKT *pkt );
void ipmi_arm_pef_postpone_timer( IPMI_PKT *pkt );
void ipmi_set_pef_config_params( IPMI_PKT *pkt );
void ipmi_get_pef_config_params( IPMI_PKT *pkt );
void ipmi_set_last_processed_event( IPMI_PKT *pkt );
void ipmi_get_last_processed_event( IPMI_PKT *pkt );   
void ipmi_platform_event( IPMI_PKT *pkt );
void ipmi_set_event_receiver( IPMI_PKT *pkt );
void ipmi_get_event_receiver( IPMI_PKT *pkt );
int event_data_compare( uchar test_value, PEF_MASK *pef_mask );
int ipmi_send_event_req( uchar *msg_cmd, unsigned msg_len, void(*ipmi_completion_function)( void *, int ) );
