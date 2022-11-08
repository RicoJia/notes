<?php
/**
 * The template for displaying product content within loops
 */

defined( 'ABSPATH' ) || exit;

global $product;

// Ensure visibility.
if ( empty( $product ) || ! $product->is_visible() ) {
	return;
}
?>
<li <?php wc_product_class( '', $product ); ?>>
	<div class="product">
		<div class="product-inner">
			<div class="product-img">
				<?php
				/**
				 * Hook: woocommerce_before_shop_loop_item.
				 *
				 * @hooked woocommerce_template_loop_product_link_open - 10
				 */
				do_action( 'woocommerce_before_shop_loop_item' );
				?>
				<a href="<?php echo esc_url(the_permalink()); ?>">
					<?php the_post_thumbnail(); ?>
				</a>
				<?php if ( $product->is_on_sale() ) : ?>
					<?php echo apply_filters( 'woocommerce_sale_flash', '<span class="badge">' . esc_html__( 'Sale', 'appetizer' ) . '</span>', $post, $product ); ?>
				<?php endif; ?>
			</div>
			<div class="product-content">
				<div class="price">
					<?php echo $product->get_price_html(); ?>
				</div>
				<h5><a href="<?php echo esc_url(the_permalink()); ?>"><?php the_title(); ?></a></h5>
				<p><?php the_excerpt(); ?></p>
				<div class="product-action">
					<?php

					/**
					 * Hook: woocommerce_after_shop_loop_item.
					 *
					 * @hooked woocommerce_template_loop_product_link_close - 5
					 * @hooked woocommerce_template_loop_add_to_cart - 10
					 */
					do_action( 'woocommerce_after_shop_loop_item' );
					?>
				</div>
			</div>
		</div>
	</div>
</li>
