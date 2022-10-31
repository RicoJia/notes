<?php
class appetizer_import_dummy_data {

	private static $instance;

	public static function init( ) {
		if ( ! isset( self::$instance ) && ! ( self::$instance instanceof appetizer_import_dummy_data ) ) {
			self::$instance = new appetizer_import_dummy_data;
			self::$instance->appetizer_setup_actions();
		}

	}

	/**
	 * Setup the class props based on the config array.
	 */
	

	/**
	 * Setup the actions used for this class.
	 */
	public function appetizer_setup_actions() {

		// Enqueue scripts
		add_action( 'customize_controls_enqueue_scripts', array( $this, 'appetizer_import_customize_scripts' ), 0 );

	}
	
	

	public function appetizer_import_customize_scripts() {

	wp_enqueue_script( 'appetizer-import-customizer-js', APPETIZER_PARENT_INC_URI . '/customizer/customizer-notify/js/appetizer-import-customizer-options.js', array( 'customize-controls' ) );
	}
}

$appetizer_import_customizers = array(

		'import_data' => array(
			'recommended' => true,
			
		),
);
appetizer_import_dummy_data::init( apply_filters( 'appetizer_import_customizer', $appetizer_import_customizers ) );