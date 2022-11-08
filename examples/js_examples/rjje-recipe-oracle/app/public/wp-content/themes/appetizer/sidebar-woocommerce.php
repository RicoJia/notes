<?php
/**
 * side bar template
 */
?>
<?php if ( ! is_active_sidebar( 'appetizer-woocommerce-sidebar' ) ) {	return; } ?>
<div class="col-lg-4 col-12">
	<div class="sidebar">
		<?php dynamic_sidebar('appetizer-woocommerce-sidebar'); ?>
	</div>
</div>