<?php
/* Notifications in customizer */


require APPETIZER_PARENT_INC_DIR . '/customizer/customizer-notify/appetizer-notify.php';
$appetizer_config_customizer = array(
	'recommended_plugins'       => array(
		'burger-companion' => array(
			'recommended' => true,
			'description' => sprintf(__('Install and activate <strong>Burger Companion</strong> plugin for taking full advantage of all the features this theme has to offer Appetizer.', 'appetizer')),
		),
	),
	'recommended_actions'       => array(),
	'recommended_actions_title' => esc_html__( 'Recommended Actions', 'appetizer' ),
	'recommended_plugins_title' => esc_html__( 'Recommended Plugin', 'appetizer' ),
	'install_button_label'      => esc_html__( 'Install and Activate', 'appetizer' ),
	'activate_button_label'     => esc_html__( 'Activate', 'appetizer' ),
	'appetizer_deactivate_button_label'   => esc_html__( 'Deactivate', 'appetizer' ),
);
Appetizer_Customizer_Notify::init( apply_filters( 'appetizer_customizer_notify_array', $appetizer_config_customizer ) );
