( function( api ) {

	// Extends our custom "appetizer" section.
	api.sectionConstructor['appetizer'] = api.Section.extend( {

		// No events for this type of section.
		attachEvents: function () {},

		// Always make the section active.
		isContextuallyActive: function () {
			return true;
		}
	} );

} )( wp.customize );