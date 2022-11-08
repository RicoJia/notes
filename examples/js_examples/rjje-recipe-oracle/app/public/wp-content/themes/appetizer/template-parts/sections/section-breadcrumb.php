<?php 
	$appetizer_hs_breadcrumb			= get_theme_mod('hs_breadcrumb','1');
	$appetizer_breadcrumb_bg_img		= get_theme_mod('breadcrumb_bg_img',esc_url(get_template_directory_uri() .'/assets/images/breadcrumbg.jpg')); 
	$appetizer_breadcrumb_back_attach	= get_theme_mod('breadcrumb_back_attach','scroll');
		
if($appetizer_hs_breadcrumb == '1') {	?>
	<section id="breadcrumb-section" class="breadcrumb-area breadcrumb-center" style="background: url('<?php echo esc_url($appetizer_breadcrumb_bg_img); ?>') center center <?php echo esc_attr($appetizer_breadcrumb_back_attach); ?>;">
            <div class="container">
                <div class="row">
                    <div class="col-12">
						<div class="breadcrumb-content">
							<div class="breadcrumb-heading">
								<h2>
									<?php appetizer_breadcrumb_title();	?>
								</h2>
							</div>
						
							<ol class="breadcrumb-list">
								<?php appetizer_breadcrumbs(); ?>
							</ol>
						</div>  					
                    </div>
                </div>
            </div>
        </section>
<?php }else{ ?>	
<section id="breadcrumb-section" class="breadcrumb-area breadcrumb-center">
</section>
<?php } ?>