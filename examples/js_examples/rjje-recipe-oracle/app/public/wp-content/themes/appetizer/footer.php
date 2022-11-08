</div>
 <footer id="footer-section" class="footer-section main-footer">
		<?php do_action('appetizer_footer_above'); ?>
		<div class="footer-main">
			<?php  if ( is_active_sidebar( 'appetizer-footer-widget-area' ) ) { ?>		
				<div class="container">
					<div class="row g-4">
						<?php  dynamic_sidebar( 'appetizer-footer-widget-area' ); ?>
					</div>
				</div>
			<?php  } ?>	
		</div>
		<?php
			$footer_copyright	    = get_theme_mod('footer_copyright','Copyright &copy; [current_year] [site_title] | Powered by [theme_author]');
			$hs_footer_btm_support	= get_theme_mod('hs_footer_btm_support','1');
			$footer_btm_support_icon= get_theme_mod('footer_btm_support_icon','fa-phone');
			$footer_btm_support_ttl	= get_theme_mod('footer_btm_support_ttl');
			$footer_btm_support_text= get_theme_mod('footer_btm_support_text');
			$hs_footer_payment	    = get_theme_mod('hs_footer_payment','1');
			if(!empty($footer_copyright) || ($hs_footer_btm_support=='1') || ($hs_footer_payment=='1')){
		?>	
        <div class="footer-copyright">
            <div class="container">
                <div class="row align-items-center gy-lg-0 gy-4">
                    <div class="col-lg-4 col-md-6 col-12 text-lg-left text-md-left text-center">
                        <div class="widget-left text-lg-left text-md-left text-center">
							<?php if ( ! empty( $footer_copyright ) ){ 				
									$appetizer_copyright_allowed_tags = array(
										'[current_year]' => date_i18n('Y'),
										'[site_title]'   => get_bloginfo('name'),
										'[theme_author]' => sprintf(__('<a href="#">Appetizer</a>', 'appetizer')),
									);
								?>                          
								<div class="copyright-text">
									<?php
										echo apply_filters('appetizer_footer_copyright', wp_kses_post(appetizer_str_replace_assoc($appetizer_copyright_allowed_tags, $footer_copyright)));
									?>
								</div>
							<?php } ?>
                        </div>
                    </div>
                    <div class="col-lg-4 col-md-6 col-12 text-lg-center text-md-center text-center">
						<?php if($hs_footer_btm_support=='1'){ ?>
							<div class="widget-center text-lg-center text-md-center text-center">
								<aside class="widget widget-contact">
									<div class="contact-area">
										<?php if(!empty($footer_btm_support_icon)): ?>
											<div class="contact-icon">
												<div class="contact-corn"><i class="fa <?php echo esc_attr($footer_btm_support_icon); ?>"></i></div>
											</div>
										<?php endif; ?>	
										
										<?php if(!empty($footer_btm_support_ttl) || !empty($footer_btm_support_text)): ?>
											<div class="contact-info">
												<h6 class="title"><?php echo wp_kses_post($footer_btm_support_ttl); ?></h6>
												<p class="text"><?php echo wp_kses_post($footer_btm_support_text); ?></p>
											</div>
										<?php endif; ?>
									</div>
								</aside>
							</div>
						<?php } ?>
                    </div>
                    <div class="col-lg-4 col-md-6 col-12 text-lg-right text-md-right text-center">
						<?php do_action('appetizer_footer_payment_icon'); ?>
                    </div>
                </div>
            </div>
        </div>
		<?php } ?>
    </footer>
    <!-- End: Footer
    =================================-->

    <!-- Scrolling Up -->
	<button type="button" class="scrollingUp scrolling-btn" aria-label="scrollingUp"><i class="fa fa-spoon"></i></button>	

</div>		
<?php wp_footer(); ?>
</body>
</html>
