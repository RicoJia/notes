<?php if ( get_header_image() ) : ?>
	<a href="<?php echo esc_url( home_url( '/' ) ); ?>" id="custom-header" rel="home">
		<img src="<?php esc_url(header_image()); ?>" width="<?php echo esc_attr( get_custom_header()->width ); ?>" height="<?php echo esc_attr( get_custom_header()->height ); ?>" alt="<?php echo esc_attr(get_bloginfo( 'title' )); ?>">
	</a>	
<?php endif; ?>
    <!--===// Start: Main Header
    =================================-->
    <header id="main-header" class="main-header">
        <!--===// Start: Header Above
        =================================-->
		<?php do_action('appetizer_above_header'); ?>
        <!--===// End: Header Top
        =================================--> 
		<!--===// Start: Navigation Wrapper
		=================================-->
		<div class="navigation-wrapper">
			<!--===// Start: Main Desktop Navigation
			=================================-->
			<div class="main-navigation-area d-none d-lg-block">
				<div class="main-navigation <?php echo esc_attr(appetizer_sticky_menu()); ?>">
					<div class="container">
						<div class="row">
							<div class="col-3 my-auto">
								<div class="logo">
									<?php appetizer_logo_site_ttl_description(); ?>
								</div>
							</div>
							<div class="col-9 my-auto">
								<nav class="navbar-area">
									<div class="main-navbar">
										<?php appetizer_header_menu_navigation(); ?>
									</div>
									<div class="main-menu-right">
										<ul class="menu-right-list">                    
											<?php appetizer_header_search(); ?>
											<?php appetizer_header_cart(); ?>
											<?php appetizer_header_button(); ?>
										</ul>                            
									</div>
								</nav>
							</div>
						</div>
					</div>
				</div>
			</div>
			<!--===// End:  Main Desktop Navigation
			=================================-->
			<!--===// Start: Main Mobile Navigation
			=================================-->
			<?php appetizer_mobile_menu(); ?>
			<!--===// End: Main Mobile Navigation
			=================================-->
		</div>
		<!--===// End: Navigation Wrapper
		=================================-->
    </header>
    <!-- End: Main Header
    =================================-->